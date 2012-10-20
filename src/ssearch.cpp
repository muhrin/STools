/*
 * ssearch.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "STools.h"

#include <boost/exception/diagnostic_information.hpp>
#include <boost/program_options.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/tokenizer.hpp>

#include <armadillo>

#include <yaml-cpp/yaml.h>

// From SSTbx
#include <build_cell/AtomsDescription.h>
#include <build_cell/DefaultCrystalGenerator.h>
#include <build_cell/StructureDescription.h>
#include <common/AtomSpeciesDatabase.h>
#include <factory/FactoryError.h>
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

struct InputOptions
{
  unsigned int      numRandomStructures;
  double            optimisationPressure;
  ::std::string     potential;
  ::std::vector< ::std::string> potSpecies;
  ::std::vector< ::std::string> potParams;
  ::std::string     potCombiningRule;
  ::std::string     structurePath;
};

::sstbx::potential::SimplePairPotential::CombiningRule
getCombiningRuleFromString(const ::std::string & str)
{
  ::sstbx::potential::SimplePairPotential::CombiningRule rule = ::sstbx::potential::SimplePairPotential::NONE;

  if(str == "lorentz")
  {
    rule = ::sstbx::potential::SimplePairPotential::LORENTZ;
  }
  else if(str == "berthelot")
  {
    rule = ::sstbx::potential::SimplePairPotential::BERTHELOT;
  }
  else if(str == "lorentz_berthelot")
  {
    rule = ::sstbx::potential::SimplePairPotential::LORENTZ_BERTHELOT;
  }
  else if(str == "custom")
  {
    rule = ::sstbx::potential::SimplePairPotential::CUSTOM;
  }

  return rule;
}

int main(const int argc, const char * const argv[])
{
  namespace po    = ::boost::program_options;
  namespace sp    = ::spipe;
  namespace spb   = ::spipe::blocks;
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
  InputOptions in;

  try
  {
    po::options_description general("STools\nUsage: " + exeName + " [options] inpue_file...\nOptions");
    general.add_options()
      ("help", "Show help message")
      ("num,n", po::value<unsigned int>(&in.numRandomStructures)->default_value(100), "Number of random starting structures")
      ("pot,u", po::value < ::std::string>(&in.potential)->default_value("lj"), "The potential to use (possible values: lj)")
      ("input,i", po::value< ::std::string>(&in.structurePath), "The input structure")
    ;

    po::options_description lennardJones("Lennard-Jones options (when --pot lj is used)");
    lennardJones.add_options()
      ("species,s", po::value< ::std::vector< ::std::string> >(&in.potSpecies)->multitoken()->required(), "List of species the potential applies to")
      ("params,p", po::value< ::std::vector< ::std::string> >(&in.potParams)->multitoken()->required(), "Potential parameters, must be in quotes: eAA eAB eBB sAA sAB sBB beta [+/-1]")
      ("comb,c", po::value< ::std::string>(&in.potCombiningRule)->default_value("none"), "Off-diagonal combining rule to use")
      ("opt-press", po::value<double>(&in.optimisationPressure)->default_value(0.01), "Pressure used during initial optimisation step to bring atoms together")
    ;

    po::positional_options_description p;
    p.add("input", 1);

    po::options_description cmdLineOptions;
    cmdLineOptions.add(general).add(lennardJones);

    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).options(cmdLineOptions).positional(p).run(), vm);

    // Deal with help first, otherwise missing required parameters will cause exception on vm.notify
    if(vm.count("help"))
    {
      ::std::cout << cmdLineOptions << ::std::endl;
      return 1;
    }

    po::notify(vm);
  }
  catch(std::exception& e)
  {
    ::std::cout << e.what() << "\n";
    return 1;
  }

  if(in.potential != "lj")
  {
    ::std::cout << "--pot must have the value lj";
    return 1;
  }

  if(in.potParams.size() != 7)
  {
    ::std::cout << "There must be 7 potential parameters specified\n";
    return 1;
  }

  // Do potential parameters ////////////////////////////////
  vec from(8), stepSize(8);
  Col<unsigned int> numSteps(8);
  // Initialise with reasonable values
  from.zeros();
  stepSize.ones();
  numSteps.ones();

  double lFrom, lStepSize;
  unsigned int lNumSteps;
  for(size_t i = 0; i < 6; ++i)
  {
    try
    {
      sp::common::parseParamString(in.potParams[i], lFrom, lStepSize, lNumSteps);
      from(i) = lFrom;
      stepSize(i) = lStepSize;
      numSteps(i) = lNumSteps;
    }
    catch(const ::std::invalid_argument & e)
    {
      ::std::cout << "Unable to parse parameter " << i << ": " << in.potParams[i] << std::endl;
      ::std::cout << e.what();
      return 1;
    }
  }
  const double betaDiagonal = ::boost::lexical_cast<double>(in.potParams[6]);

  bool doingSweep = false;
  for(size_t i = 0; i < numSteps.n_rows; ++i)
  {
    doingSweep |= numSteps(i) != 1;
  }

  // Generate the pipeline
  typedef ::pipelib::SingleThreadedPipeline<sp::StructureDataTyp, sp::SharedDataTyp> Pipeline;
  typedef spb::PotentialParamSweep ParamSweepBlock;

  ::boost::scoped_ptr<Pipeline> potparamsSweepPipe;
  ::boost::scoped_ptr<Pipeline> randomSearchPipeOwned;

  Pipeline * masterPipe;
  Pipeline * randomSearchPipe;
  if(doingSweep)
  {
    potparamsSweepPipe.reset(new Pipeline());
    potparamsSweepPipe->getSharedData().setPipe(*potparamsSweepPipe.get());
    masterPipe = potparamsSweepPipe.get();
    randomSearchPipe = &potparamsSweepPipe->spawnChild();
  }
  else
  {
    randomSearchPipeOwned.reset(new Pipeline());
    randomSearchPipe = randomSearchPipeOwned.get();
    masterPipe = randomSearchPipe;
  }
  // Make sure the shared data is correctly hooked up
  randomSearchPipe->getSharedData().setPipe(*randomSearchPipe);

  ssc::AtomSpeciesDatabase & speciesDb = randomSearchPipe->getGlobalData().getSpeciesDatabase();

  // Do atom species ///////////////////////////////////////
  if(in.potSpecies.size() != 2)
  {
    ::std::cout << "There must be 2 potential species specified\n";
    return 1;
  }

  ::std::vector< ssc::AtomSpeciesId::Value > potentialSpecies(2);
  potentialSpecies[0] = speciesDb.getIdFromSymbol(in.potSpecies[0]);
  potentialSpecies[1] = speciesDb.getIdFromSymbol(in.potSpecies[1]);
  from(6) = potentialSpecies[0].ordinal();
  from(7) = potentialSpecies[1].ordinal();

  ssf::SsLibFactoryYaml factory(speciesDb);

  ssbc::StructureDescriptionPtr strDesc;
  try
  {
    YAML::Node loadedNode = YAML::LoadFile(in.structurePath);
    if(!loadedNode[kw::RANDOM_STRUCTURE])
    {
      ::std::cout << "Couldn't find random structure in input file\n";
      return 1;
    }
    strDesc = factory.createStructureDescription(loadedNode[kw::RANDOM_STRUCTURE]);
  }
  catch(const ssf::FactoryError & e)
  {
    ::std::cout << ::boost::diagnostic_information(e) << ::std::endl;
    return 1;
  }

  // Random structure
  ssbc::DefaultCrystalGenerator strGen(true /*use extrusion method*/);
  sp::blocks::RandomStructure randStr(strGen, in.numRandomStructures, sp::blocks::RandomStructure::StructureDescPtr(strDesc.release()));

  // Niggli reduction
  sp::blocks::NiggliReduction niggli;

  // Geometry optimise
  ::arma::mat epsilon;
	epsilon.set_size(2, 2);
	epsilon << from(0) << from(1) << endr
			<< from(1) << from(2) << endr;

	::arma::mat sigma;
	sigma.set_size(2, 2);
	sigma << from(3) << from(4) << endr
			<< from(4) << from(5) << endr;

	::arma::mat beta;
	beta.set_size(2, 2);
	beta << betaDiagonal << 1 << endr
			<< 1 << betaDiagonal << endr;


  const ssp::SimplePairPotential::CombiningRule combRule = getCombiningRuleFromString(in.potCombiningRule);

  ssp::SimplePairPotential pp(
    speciesDb,
    2,
    potentialSpecies,
    epsilon,
    sigma,
    2.5,
    beta,
    12,
    6,
    combRule
  );
  ssp::TpsdGeomOptimiser optimiser(pp);

  ::arma::mat33 optimisationPressureMtx;
  optimisationPressureMtx.fill(0.0);
  optimisationPressureMtx.diag().fill(in.optimisationPressure);

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
  randomSearchPipe->setStartBlock(randStr);
  randomSearchPipe->connect(randStr, niggli);
  randomSearchPipe->connect(niggli, goPressure);
  randomSearchPipe->connect(goPressure, go);
  randomSearchPipe->connect(go, remDuplicates);
  randomSearchPipe->connect(remDuplicates, sg);
  randomSearchPipe->connect(sg, write1);
  randomSearchPipe->connect(write1, barrier);
  randomSearchPipe->connect(barrier, write2);
  randomSearchPipe->connect(write2, lowestE);


  ::boost::scoped_ptr<ParamSweepBlock> paramSweepBlock;
  if(doingSweep)
  {
    // Configure parameter sweep pipeline
    paramSweepBlock.reset(new ParamSweepBlock(from, stepSize, numSteps, *randomSearchPipe));
    potparamsSweepPipe->setStartBlock(*paramSweepBlock.get());
  }


  // Finally initialise and start whichever pipeline is the master
  masterPipe->initialise();
  masterPipe->start();
}

