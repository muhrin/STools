/*
 * ssearch.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "STools.h"

#include <boost/program_options.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/tokenizer.hpp>

#include <armadillo>

#include <yaml-cpp/yaml.h>

// From SSTbx
#include <build_cell/AtomsDescription.h>
#include <build_cell/DefaultCrystalGenerator.h>
#include <build_cell/StructureDescription.h>
#include <factory/SsLibFactoryYaml.h>
#include <factory/SsLibYamlKeywords.h>
#include <io/ResReaderWriter.h>
#include <io/StructureWriterManager.h>
#include <potential/SimplePairPotential.h>
#include <potential/TpsdGeomOptimiser.h>
#include <utility/SortedDistanceComparator.h>
#include <utility/UniqueStructureSet.h>

// From Pipelib
#include <pipelib/IPipeline.h>
#include <pipelib/SingleThreadedPipeline.h>
#include <pipelib/DefaultBarrier.h>

// From StructurePipe
#include <StructurePipe.h>
#include <blocks/DetermineSpaceGroup.h>
#include <blocks/LowestFreeEnergy.h>
#include <blocks/NiggliReduction.h>
#include <blocks/ParamPotentialGo.h>
#include <blocks/PotentialGo.h>
#include <blocks/PotentialParamSweep.h>
#include <blocks/RandomStructure.h>
#include <blocks/RemoveDuplicates.h>
#include <blocks/StoichiometrySearch.h>
#include <blocks/WriteStructure.h>
#include <common/SharedData.h>
#include <common/StructureData.h>
#include <common/UtilityFunctions.h>

// MACROS ////////////////////////////////////

// NAMESPACES ////////////////////////////////


int main(const int argc, const char * const argv[])
{
  namespace po    = ::boost::program_options;
  namespace sp    = ::spipe;
  namespace ssbc  = ::sstbx::build_cell;
  namespace ssc   = ::sstbx::common;
  namespace ssf   = ::sstbx::factory;
  namespace ssio  = ::sstbx::io;
  namespace ssp   = ::sstbx::potential;
  namespace ssu   = ::sstbx::utility;
  namespace kw    = ssf::sslib_yaml_keywords;
  using ::arma::Mat;
  using ::arma::Col;
  using ::arma::vec;
  using ::arma::endr;

  typedef boost::tokenizer<boost::char_separator<char> > Tok;
  const boost::char_separator<char> tokSep(" \t");

  const ::std::string exeName(argv[0]);

  // Program options
  ::std::string paramsString;
  unsigned int numRandomStructures;
  double optimisationPressure;
  ::std::string inputStructure;

  try
  {
    po::options_description desc("STools\nUsage: " + exeName + " [options] inpue_file...\nOptions");
    desc.add_options()
      ("help", "Show help message")
      ("num,n", po::value<unsigned int>(&numRandomStructures)->default_value(100), "Number of random starting structures")
      ("params,p", po::value< ::std::string>(&paramsString)->required(), "potential parameters, must be in quotes: eAA eAB eBB sAA sAB sBB beta [+/-1]")
      ("opt-press", po::value<double>(&optimisationPressure)->default_value(0.01), "Pressure used during initial optimisation step to bring atoms together")
      ("input", po::value< ::std::string>(&inputStructure), "The input structure")
    ;

    po::positional_options_description p;
    p.add("input", 1);

    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).options(desc).positional(p).run(), vm);

    // Deal with help first, otherwise missing required parameters will cause exception on vm.notify
    if(vm.count("help"))
    {
      ::std::cout << desc << ::std::endl;
      return 1;
    }

    po::notify(vm);
  }
  catch(std::exception& e)
  {
    ::std::cout << e.what() << "\n";
    return 1;
  }


  // Potential parameters
  vec params(8);
  double betaDiagonal;
  Tok toker(paramsString, tokSep);

  size_t i = 0;
  bool parsedSuccessfully = true;
  BOOST_FOREACH(::std::string paramStr, toker)
  {
    if(i == 7)
    {
      parsedSuccessfully = false;
      break;
    }
    try
    {
      if(i == 6)
      {
        betaDiagonal = ::boost::lexical_cast<double>(paramStr);
      }
      else
      {
        params(i) = ::boost::lexical_cast<double>(paramStr);
      }
    }
    catch(::boost::bad_lexical_cast )
    {
      parsedSuccessfully = false;
      break;
    }
    ++i;
  }

  if(!parsedSuccessfully)
  {
    ::std::cout << "Error parsing parameters\n";
    return 1;
  } 


  ::std::vector< ssc::AtomSpeciesId::Value > potentialSpecies(2);
  potentialSpecies[0] = ssc::AtomSpeciesId::NA;
  potentialSpecies[1] = ssc::AtomSpeciesId::CL;

  ssf::SsLibFactoryYaml factory;

  YAML::Node loadedNode = YAML::LoadFile(inputStructure);

  YAML::Node * structureNode = &loadedNode;
  YAML::Node strNode;
  if(loadedNode[kw::STRUCTURE])
  {
    strNode = loadedNode[kw::STRUCTURE];
    structureNode = &strNode;
  }

  ssbc::StructureDescriptionPtr strDesc = factory.createStructureDescription(*structureNode); 

  // Generate the pipelines that we need
  ::pipelib::SingleThreadedPipeline<sp::StructureDataTyp, sp::SharedDataTyp> randomSearchPipe;

  // Make sure the shared data is correctly hooked up
  randomSearchPipe.getSharedData().setPipe(randomSearchPipe);

  // Random structure
  ssbc::DefaultCrystalGenerator strGen(true /*use extrusion method*/);
  sp::blocks::RandomStructure randStr(strGen, numRandomStructures, sp::blocks::RandomStructure::StructureDescPtr(strDesc.release()));

  // Niggli reduction
  sp::blocks::NiggliReduction niggli;

  // Geometry optimise
  ::arma::mat epsilon;
	epsilon.set_size(2, 2);
	epsilon << params(0) << params(1) << endr
			<< params(1) << params(2) << endr;

	::arma::mat sigma;
	sigma.set_size(2, 2);
	sigma << params(3) << params(4) << endr
			<< params(4) << params(5) << endr;

	::arma::mat beta;
	beta.set_size(2, 2);
	beta << params(6) << 1 << endr
			<< 1 << params(6) << endr;

  ssp::SimplePairPotential pp(
    2,
    potentialSpecies,
    epsilon,
    sigma,
    2.5,
    beta,
    12,
    6,
    ssp::SimplePairPotential::CUSTOM
  );
  ssp::TpsdGeomOptimiser optimiser(pp);

  ::arma::mat33 optimisationPressureMtx;
  optimisationPressureMtx.fill(0.0);
  optimisationPressureMtx.diag().fill(optimisationPressure);

  sp::blocks::ParamPotentialGo goPressure(pp, optimiser, &optimisationPressureMtx, false);

  sp::blocks::ParamPotentialGo go(pp, optimiser, NULL /*no external pressure*/, true);

  // Remove duplicates
  ssu::SortedDistanceComparator comparator;
  ssu::UniqueStructureSet uniqueSet(comparator);
  sp::blocks::RemoveDuplicates remDuplicates(uniqueSet);

  // Determine space group
  sp::blocks::DetermineSpaceGroup sg;

  // Write structures
  ssio::ResReaderWriter resIo;
  ssio::StructureWriterManager writerManager;
  writerManager.registerWriter(resIo);
  sp::blocks::WriteStructure write1(writerManager);

  // Barrier
  ::pipelib::DefaultBarrier<sp::StructureDataTyp, sp::SharedDataTyp> barrier;

  // Write structures 2
  sp::blocks::WriteStructure write2(writerManager);

  // Get the lowest free energy
  sp::blocks::LowestFreeEnergy lowestE;

  // Put it all together
  randomSearchPipe.setStartBlock(randStr);
  randomSearchPipe.connect(randStr, niggli);
  randomSearchPipe.connect(niggli, goPressure);
  randomSearchPipe.connect(goPressure, go);
  randomSearchPipe.connect(go, remDuplicates);
  randomSearchPipe.connect(remDuplicates, sg);
  randomSearchPipe.connect(sg, write1);
  randomSearchPipe.connect(write1, barrier);
  randomSearchPipe.connect(barrier, write2);
  randomSearchPipe.connect(write2, lowestE);

  randomSearchPipe.initialise();
  randomSearchPipe.start();
}

