/*
 * stoich.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "STools.h"

#include <boost/program_options.hpp>
#include <boost/scoped_ptr.hpp>

#include <armadillo>

// From SSTbx
#include <build_cell/AtomsDescription.h>
#include <build_cell/DefaultCrystalGenerator.h>
#include <build_cell/StructureDescription.h>
#include <io/ResReaderWriter.h>
#include <io/StructureWriterManager.h>
#include <potential/SimplePairPotential.h>
#include <potential/TpsdGeomOptimiser.h>
#include <utility/SortedDistanceComparator.h>
#include <utility/UniqueStructureSet.h>

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
  namespace ssio  = ::sstbx::io;
  namespace ssp   = ::sstbx::potential;
  namespace ssu   = ::sstbx::utility;
  using ::arma::Mat;
  using ::arma::Col;
  using ::arma::vec;
  using ::arma::endr;
  using ::std::cout;
  using ::std::endl;

  const ::std::string exeName(argv[0]);

  // Program options
  ::std::vector< ::std::string> paramStrings;
  unsigned int maxNumAtoms;
  unsigned int numRandomStructures;

  try
  {
    po::options_description desc("Usage: " + exeName + " [options] params...\nOptions");
    desc.add_options()
      ("help", "Show help message")
      ("strs", po::value<unsigned int>(&numRandomStructures)->default_value(100), "Number of random starting structures")
      ("max-atoms,m", po::value<unsigned int>(&maxNumAtoms)->required(), "Maximum number of atoms")
      ("params", po::value< ::std::vector< ::std::string> >(&paramStrings)->required(), "potential parameters: eAA eAB eBB sAA sAB sBB [format from+delta*nsteps] beta [+/-1]")

    ;

    po::positional_options_description p;
    p.add("params", 7);

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

  // Param sweep
  vec from(8), step(8);
  Col<unsigned int> steps(8);

  from.fill(0.0);
  step.fill(0.0);
  steps.fill(0);

  double lFrom, lStep;
  unsigned int lNSteps;
  bool parsedParams = true;
  for(size_t i = 0; i < 6; ++i)
  {
    if(sp::common::parseParamString(paramStrings[i], lFrom, lStep, lNSteps))
    {
      from(i) = lFrom;
      step(i) = lStep;
      steps(i) = lNSteps;
    }
    else
    {
      parsedParams = false;
      cout << "Unable to parse parameter " << i << ": " << paramStrings[i] << endl;
      return 1;
    }
  }

  const double betaDiagonal = ::boost::lexical_cast<double>(paramStrings[6]);

  ::std::vector< ssc::AtomSpeciesId::Value > potentialSpecies(2);
  potentialSpecies[0] = ssc::AtomSpeciesId::NA;
  potentialSpecies[1] = ssc::AtomSpeciesId::CL;
  from(6) = potentialSpecies[0].ordinal();
  from(7) = potentialSpecies[1].ordinal();


  // Generate the pipelines that we need
  ::pipelib::SingleThreadedPipeline<sp::StructureDataTyp, sp::SharedDataTyp> paramSweepPipe;
  ::pipelib::SingleThreadedPipeline<sp::StructureDataTyp, sp::SharedDataTyp> & stoichSweepPipe   = paramSweepPipe.spawnChild();
  ::pipelib::SingleThreadedPipeline<sp::StructureDataTyp, sp::SharedDataTyp> & randomSearchPipe  = stoichSweepPipe.spawnChild();

  // Make sure the shared data is correctly hooked up
  paramSweepPipe.getSharedData().setPipe(paramSweepPipe);
  stoichSweepPipe.getSharedData().setPipe(stoichSweepPipe);
  randomSearchPipe.getSharedData().setPipe(randomSearchPipe);

  // Random structure
  ssbc::DefaultCrystalGenerator strGen(true /*use extrusion method*/);
  sp::blocks::RandomStructure randStr(numRandomStructures, strGen);

  // Niggli reduction
  sp::blocks::NiggliReduction niggli;

  // Geometry optimise
  ::arma::mat epsilon;
	epsilon.set_size(2, 2);
	epsilon << 1 << 2 << endr
			<< 2 << 2 << endr;

	::arma::mat sigma;
	sigma.set_size(2, 2);
	sigma << 2 << 2 << endr
			<< 2 << 2 << endr;

	::arma::mat beta;
	beta.set_size(2, 2);
	beta << betaDiagonal << 1 << endr
			<< 1 << betaDiagonal << endr;

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

  ::arma::mat33 externalPressure;
  externalPressure.fill(0.0);
  externalPressure.diag().fill(0.1);

  sp::blocks::ParamPotentialGo goPressure(pp, optimiser, &externalPressure, false);

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

  // Configure stoichiometry sweep pipeline
  sp::blocks::StoichiometrySearch stoichSearch(ssc::AtomSpeciesId::NA, ssc::AtomSpeciesId::CL, maxNumAtoms, randomSearchPipe);
  sp::blocks::LowestFreeEnergy lowestEStoich;

  stoichSweepPipe.setStartBlock(stoichSearch);
  stoichSweepPipe.connect(stoichSearch, lowestEStoich);


  // Configure parameter sweep pipeline
  sp::blocks::PotentialParamSweep ppSweep(from, step, steps, stoichSweepPipe);

  paramSweepPipe.setStartBlock(ppSweep);

  paramSweepPipe.initialise();
  paramSweepPipe.start();
}

