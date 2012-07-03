/*
 * stoich.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "STools.h"

#include <boost/scoped_ptr.hpp>

#include <armadillo>

// From SSTbx
#include <build_cell/AtomsDescription.h>
#include <build_cell/DefaultCrystalGenerator.h>
#include <build_cell/RandomCellDescription.h>
#include <build_cell/RandomCellGenerator.h>
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

  // Get command line parameters
  if(argc != 9)
  {
    cout << "Usage: " << argv[0] << " [6 x params, format from+delta*nsteps] [+/-1] [max_atoms]" << endl;
    return 0;
  }

  // Param sweep
  vec from(8), step(8);
  Col<unsigned int> steps(8);

  from.fill(0);
  step.fill(0);
  step.fill(0);

  double lFrom, lStep;
  unsigned int lNSteps;
  bool parsedParams = true;
  for(size_t i = 0; i < 6; ++i)
  {
    if(sp::common::parseParamString(argv[i + 1], lFrom, lStep, lNSteps))
    {
      from(i) = lFrom;
      step(i) = lStep;
      steps(i) = lNSteps;
    }
    else
    {
      parsedParams = false;
      cout << "Unable to parse parameter " << i << ": " << argv[i + 1] << endl;
      break;
    }
  }

  if(!parsedParams)
  {
    return 0;
  }

  ::std::vector< ssc::AtomSpeciesId::Value > potentialSpecies(2);
  potentialSpecies[0] = ssc::AtomSpeciesId::NA;
  potentialSpecies[1] = ssc::AtomSpeciesId::CL;
  from(6) = potentialSpecies[0].ordinal();
  from(7) = potentialSpecies[1].ordinal();

  const double betaDiagonal = ::boost::lexical_cast<double>(argv[7]);

  const size_t maxNumAtoms = ::boost::lexical_cast<size_t>(argv[8]);


  // Generate the pipelines that we need
  ::pipelib::SingleThreadedPipeline<sp::StructureDataTyp, sp::SharedDataTyp> paramSweepPipe;
  ::pipelib::SingleThreadedPipeline<sp::StructureDataTyp, sp::SharedDataTyp> & stoichSweepPipe   = paramSweepPipe.spawnChild();
  ::pipelib::SingleThreadedPipeline<sp::StructureDataTyp, sp::SharedDataTyp> & randomSearchPipe  = stoichSweepPipe.spawnChild();

  // Make sure the shared data is correctly hooked up
  paramSweepPipe.getSharedData().setPipe(paramSweepPipe);
  stoichSweepPipe.getSharedData().setPipe(stoichSweepPipe);
  randomSearchPipe.getSharedData().setPipe(randomSearchPipe);

  // Random structure
  ssbc::RandomCellGenerator<double> cellGen;
  ssbc::DefaultCrystalGenerator strGen(cellGen);
  sp::blocks::RandomStructure randStr(100, strGen);

  // Niggli reduction
  sp::blocks::NiggliReduction niggli;

  // Geometry optimise
	Mat<double> epsilon;
	epsilon.set_size(2, 2);
	epsilon << 1 << 2 << endr
			<< 2 << 2 << endr;

	Mat<double> sigma;
	sigma.set_size(2, 2);
	sigma << 2 << 2 << endr
			<< 2 << 2 << endr;

	Mat<double> beta;
	beta.set_size(2, 2);
	beta << betaDiagonal << 1 << endr
			<< 1 << betaDiagonal << endr;

  ssp::SimplePairPotential<double> pp(
    2,
    potentialSpecies,
    epsilon,
    sigma,
    2.5,
    beta,
    12,
    6,
    ssp::SimplePairPotential<double>::CUSTOM
  );
  ssp::TpsdGeomOptimiser<double> optimiser(pp);

  ::arma::mat33 externalPressure;
  externalPressure.fill(0.0);
  externalPressure.diag().fill(0.1);

  sp::blocks::ParamPotentialGo goPressure(pp, optimiser, &externalPressure, false);

  sp::blocks::PotentialGo go(optimiser, NULL, true);

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
  sp::blocks::StoichiometrySearch stoichSearch(ssc::AtomSpeciesId::NA, ssc::AtomSpeciesId::CL, maxNumAtoms, 0.75, randomSearchPipe);
  sp::blocks::LowestFreeEnergy lowestEStoich;

  stoichSweepPipe.setStartBlock(stoichSearch);
  stoichSweepPipe.connect(stoichSearch, lowestEStoich);


  // Configure parameter sweep pipeline
  sp::blocks::PotentialParamSweep ppSweep(from, step, steps, stoichSweepPipe);

  paramSweepPipe.setStartBlock(ppSweep);

  paramSweepPipe.initialise();
  paramSweepPipe.start();
}

