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
#include <build_cell/StructureBuilder.h>
#include <io/ResReaderWriter.h>
#include <io/StructureReadWriteManager.h>
#include <potential/SimplePairPotential.h>
#include <potential/TpsdGeomOptimiser.h>
#include <utility/SortedDistanceComparator.h>
#include <utility/UniqueStructureSet.h>

#include <pipelib/pipelib.h>

// From StructurePipe
#include <StructurePipe.h>
#include <PipeLibTypes.h>
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
#include <utility/PipeDataInitialisation.h>

// Local
#include "input/YamlKeywords.h"
#include "utility/BoostCapabilities.h"

// MACROS ////////////////////////////////////

// NAMESPACES ////////////////////////////////
namespace sp = ::spipe;

struct InputOptions
{
  unsigned int      numRandomStructures;
  double            optimisationPressure;
  //::std::string     potential;
  //::std::vector< ::std::string> potSpecies;
  ::std::vector< ::std::string> potParams;
  ::std::string     structurePath;
  unsigned int      maxNumAtoms;
};

int processPotParams(
  sp::common::ParamRange & paramRange,
  double & betaDiagonal,
  const InputOptions & in);

int main(const int argc, char * argv[])
{
  namespace po    = ::boost::program_options;
  namespace sp    = ::spipe;
  namespace spu   = ::spipe::utility;
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
  InputOptions in;

  try
  {
    po::options_description desc("STools\nUsage: " + exeName + " [options] params...\nOptions");
    desc.add_options()
      ("help", "Show help message")
      //("species,s", po::value< ::std::vector< ::std::string> >(&in.potSpecies)->multitoken()_ADD_REQUIRED_, "List of species the potential applies to")
      ("params,p", po::value< ::std::vector< ::std::string> >(&in.potParams)->multitoken()_ADD_REQUIRED_, "potential parameters, must be in quotes: eAA eAB eBB sAA sAB sBB beta [+/-1]")
      ("opt-press", po::value<double>(&in.optimisationPressure)->default_value(0.01), "Pressure used during initial optimisation step to bring atoms together")
      ("num,n", po::value<unsigned int>(&in.numRandomStructures)->default_value(100), "Number of random starting structures")
      //("pot", po::value < ::std::string>(&in.potential)_ADD_REQUIRED_, "The potential to use (possible values: lj)")
      ("input", po::value< ::std::string>(&in.structurePath), "The input structure")
      ("max-atoms,m", po::value<unsigned int>(&in.maxNumAtoms)_ADD_REQUIRED_, "Maximum number of atoms")
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

  if(in.potParams.size() != 7)
  {
    ::std::cout << "Parameter string must contain 7 entries, only " << in.potParams.size() << " found\n";
    return 1;
  }

  // Do potential parameters ////////////////////////////////
  sp::common::ParamRange paramRange(8);
  double betaDiagonal;
  int result = processPotParams(paramRange, betaDiagonal, in);
  if(result != 0)
    return result;

  ::std::vector< ssc::AtomSpeciesId::Value > potentialSpecies(2);
  potentialSpecies[0] = ssc::AtomSpeciesId::NA;
  potentialSpecies[1] = ssc::AtomSpeciesId::CL;
  paramRange.from(6) = potentialSpecies[0].ordinal();
  paramRange.from(7) = potentialSpecies[1].ordinal();


  // Generate the pipelines that we need
  sp::SpSingleThreadedEngine engine;
  sp::SpEngine::RunnerPtr runner = spu::generateRunnerInitDefault(engine);
  ssc::AtomSpeciesDatabase & speciesDb = runner->memory().global().getSpeciesDatabase();

  // Random structure
  sp::blocks::RandomStructure randStr(in.numRandomStructures);

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
    speciesDb,
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

  ssp::OptimisationSettings optimisationParams;
  ::arma::mat33 pressureMtx;
  pressureMtx.zeros();
  pressureMtx.diag().fill(in.optimisationPressure);
  optimisationParams.setExternalPressure(pressureMtx);

  sp::blocks::ParamPotentialGo goPressure(pp, optimiser, optimisationParams, false);

  sp::blocks::ParamPotentialGo go(pp, optimiser, true);

  // Remove duplicates
  ssu::SortedDistanceComparator comparator;
  sp::blocks::RemoveDuplicates remDuplicates(comparator);

  // Determine space group
  sp::blocks::DetermineSpaceGroup sg;

  // Write structures
  sp::blocks::WriteStructure write1;

  // Barrier
  sp::SpSimpleBarrier barrier;

  // Write structures 2
  sp::blocks::WriteStructure write2;

  // Get the lowest free energy
  sp::blocks::LowestFreeEnergy lowestE;

  // Put it all together
  randStr |= niggli |= goPressure |= go |= remDuplicates |= sg |= write1 |= barrier |= write2 |= lowestE;

  // Configure stoichiometry sweep pipeline
  sp::blocks::StoichiometrySearch stoichSearch(ssc::AtomSpeciesId::NA, ssc::AtomSpeciesId::CL, in.maxNumAtoms, randStr);
  sp::blocks::LowestFreeEnergy lowestEStoich;

  stoichSearch |= lowestEStoich;

  // Configure parameter sweep pipeline
  sp::blocks::PotentialParamSweep ppSweep(paramRange, stoichSearch);

  runner->run();
}

int processPotParams(
  sp::common::ParamRange & paramRange,
  double & betaDiagonal,
  const InputOptions & in)
{
  // Initialise with reasonable values
  paramRange.from.zeros();
  paramRange.step.ones();
  paramRange.nSteps.ones();

  double lFrom, lStepSize;
  unsigned int lNumSteps;
  for(size_t i = 0; i < 6; ++i)
  {
    try
    {
      sp::common::parseParamString(in.potParams[i], lFrom, lStepSize, lNumSteps);
      paramRange.from(i) = lFrom;
      paramRange.step(i) = lStepSize;
      paramRange.nSteps(i) = lNumSteps;
    }
    catch(const ::std::invalid_argument & e)
    {
      ::std::cout << "Unable to parse parameter " << i << ": " << in.potParams[i] << std::endl;
      ::std::cout << e.what();
      return 1;
    }
  }

  betaDiagonal = ::boost::lexical_cast<double>(in.potParams[6]);

  // Everything went fine
  return 0;
}
