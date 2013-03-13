/*
 * ssearch.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "STools.h"

#include <boost/exception/diagnostic_information.hpp>
#include <boost/optional.hpp>
#include <boost/program_options.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/tokenizer.hpp>

#include <armadillo>

#include <yaml-cpp/yaml.h>

// From SSTbx
#include <build_cell/AtomsDescription.h>
#include <build_cell/StructureBuilder.h>
#include <common/AtomSpeciesDatabase.h>
#include <factory/FactoryError.h>
#include <factory/SsLibFactoryYaml.h>
#include <factory/SsLibYamlKeywords.h>
#include <io/StructureReadWriteManager.h>
#include <potential/SimplePairPotential.h>
#include <potential/FixedLatticeShapeConstraint.h>
#include <potential/TpsdGeomOptimiser.h>
#include <utility/SortedDistanceComparator.h>
#include <utility/UniqueStructureSet.h>
#include <yaml_schema/SchemaParse.h>

#include <pipelib/pipelib.h>

// From StructurePipe
#include <StructurePipe.h>
#include <SpTypes.h>
#include <blocks/DetermineSpaceGroup.h>
#include <blocks/LoadSeedStructures.h>
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

// Local
#include "input/YamlKeywords.h"
#include "utility/BoostCapabilities.h"
#include "utility/PipeDataInitialisation.h"
#include "factory/StFactory.h"
#include "factory/YamlSchema.h"

// MACROS ////////////////////////////////////

// NAMESPACES ////////////////////////////////
namespace po    = ::boost::program_options;
namespace sp    = ::spipe;
namespace spu   = ::spipe::utility;
namespace spb   = ::spipe::blocks;
namespace ssbc  = ::sstbx::build_cell;
namespace ssc   = ::sstbx::common;
namespace ssf   = ::sstbx::factory;
namespace ssio  = ::sstbx::io;
namespace ssp   = ::sstbx::potential;
namespace ssu   = ::sstbx::utility;
namespace ssys  = ::sstbx::yaml_schema;
namespace yaml_kw = ::stools::input::yaml_kw;

// CLASSES //////////////////////////////////
struct InputOptions
{
  unsigned int      numRandomStructures;
  double            optimisationPressure;
  ::std::string     potential;
  ::std::vector< ::std::string> potSpecies;
  ::std::vector< ::std::string> potParams;
  ::std::string     potCombiningRule;
  ::std::string     structurePath;
  double            potCutoff;
};

struct InputType
{
  enum Value { UNKNOWN, RANDOM_STRUCTURES, SEED_STRUCTURES };
};

struct RunConfiguration
{
  RunConfiguration(ssc::AtomSpeciesDatabase & speciesDb_):
    speciesDb(speciesDb_),
    paramRange(8),
    potentialSpecies(2),
    stoichMaxAtoms(0)
  {}

  ssc::AtomSpeciesDatabase & speciesDb;
  InputType::Value inputType;
  ::boost::scoped_ptr<sp::SpStartBlock> searchStartBlock;

  ::boost::scoped_ptr<sp::SpStartBlock> paramSweepBlock;
  bool sweepMode;
  sp::common::ParamRange paramRange;
  double betaDiagonal;
  ::std::vector<ssc::AtomSpeciesId::Value> potentialSpecies;

  ::std::vector<ssc::AtomSpeciesId::Value> stoichSpecies;
  unsigned int stoichMaxAtoms;
  ::boost::scoped_ptr< ::spipe::blocks::StoichiometrySearch> stoichSearchBlock;
};

// CONSTANTS /////////////////////////////////
static const double DEFAULT_INITIAL_OPTIMISATION_MAX_ITERS = 5000;
static const double DEFAULT_OPTIMISATION_MAX_ITERS = 10000;
static const double DEFAULT_POT_CUTOFF = 2.5;

// FUNCTIONS ////////////////////////////////
int parseInput(RunConfiguration & config, const InputOptions & in);

bool requiredNode(const YAML::Node & parent, const ::std::string & childName);

::std::vector<ssc::AtomSpeciesId> getSpeciesFromString(
  const ::std::string & speciesString,
  const ssc::AtomSpeciesDatabase & speciesDb
);

int processCommandLineArgs(InputOptions & in, const int argc, char * argv[]);

int processPotParams(
  sp::common::ParamRange & paramRange,
  double & betaDiagonal,
  const InputOptions & in);

int main(const int argc, char * argv[])
{
  using ::arma::Mat;
  using ::arma::Col;
  using ::arma::vec;
  using ::arma::endr;


  // Program options
  InputOptions in;

  int result = processCommandLineArgs(in, argc, argv);
  //if(result != 0)
  //  return result;

  ssys::SchemaParse parse;
  const YAML::Node searchNode = YAML::LoadFile(in.structurePath);
  stools::factory::Search searchSchema;
  ssu::HeterogeneousMap schemaOptions;
  searchSchema.nodeToValue(parse, schemaOptions, searchNode, true);
  if(parse.hasErrors())
  {
    parse.printErrors();
    return 1;
  }

  ssc::AtomSpeciesDatabase speciesDb;

  typedef ::sstbx::UniquePtr< ::spipe::SpPipe>::Type PipePtr;
  ::stools::factory::Factory factory(speciesDb);

  PipePtr pipe;
  factory.createSearchPipe(pipe, schemaOptions);

 // // Generate the pipeline
 // typedef sp::SpSingleThreadedEngine Engine;
 // typedef Engine::RunnerPtr RunnerPtr;
 // typedef spb::PotentialParamSweep ParamSweepBlock;

 // Engine pipeEngine;
 // RunnerPtr runner = spu::generateRunnerInitDefault(pipeEngine);

 // // Generate the run configuration
 // RunConfiguration config(runner->memory().global().getSpeciesDatabase());
 // result = parseInput(config, in);
 // if(result != 0)
 //   return result;

 // ::boost::scoped_ptr<sp::SpStartBlock> potparamsSweepPipe;
 // ::boost::scoped_ptr<sp::SpStartBlock> randomSearchPipeOwned;

 // // Niggli reduction
 // sp::blocks::NiggliReduction niggli;

 // // Geometry optimise
 // ::arma::mat epsilon;
	//epsilon.set_size(2, 2);
	//epsilon << config.paramRange.from(0) << config.paramRange.from(1) << endr
	//		<< config.paramRange.from(1) << config.paramRange.from(2) << endr;

	//::arma::mat sigma;
	//sigma.set_size(2, 2);
	//sigma << config.paramRange.from(3) << config.paramRange.from(4) << endr
	//		<< config.paramRange.from(4) << config.paramRange.from(5) << endr;

	//::arma::mat beta;
	//beta.set_size(2, 2);
	//beta << config.betaDiagonal << 1 << endr
	//		<< 1 << config.betaDiagonal << endr;

 // const ssp::SimplePairPotential::CombiningRule combRule = getCombiningRuleFromString(in.potCombiningRule);

 // ssp::IPotentialPtr pp(new ssp::SimplePairPotential(
 //   config.speciesDb,
 //   config.potentialSpecies,
 //   epsilon,
 //   sigma,
 //   in.potCutoff,
 //   beta,
 //   12,
 //   6,
 //   combRule
 // ));
 // ssp::TpsdGeomOptimiser optimiser(pp);

 // ssp::OptimisationSettings initialOptimisationParams;
 // ::arma::mat33 pressureMtx;
 // initialOptimisationParams.pressure = ::arma::zeros< ::arma::mat>(3, 3);
 // initialOptimisationParams.pressure->diag().fill(in.optimisationPressure);
 // initialOptimisationParams.maxSteps = static_cast<unsigned int>(DEFAULT_INITIAL_OPTIMISATION_MAX_ITERS);

 // // For seed structure pre-optimise only the lattice
 // if(config.inputType == InputType::SEED_STRUCTURES)
 //   initialOptimisationParams.optimisationType = ssp::OptimisationSettings::Optimise::LATTICE;

 // sp::blocks::ParamPotentialGo goPressure(optimiser, initialOptimisationParams, false);

 // ssp::OptimisationSettings optimisationParams;
 // optimisationParams.maxSteps = static_cast<unsigned int>(DEFAULT_OPTIMISATION_MAX_ITERS);
 // sp::blocks::ParamPotentialGo go(optimiser, optimisationParams, true);

 // // Remove duplicates
 // ssu::SortedDistanceComparator comparator;
 // sp::blocks::RemoveDuplicates remDuplicates(comparator);

 // // Determine space group
 // sp::blocks::DetermineSpaceGroup sg;

 // // Write structures
 // sp::blocks::WriteStructure write1;

 // // Barrier
 // sp::SpSimpleBarrier barrier;

 // // Write structures 2
 // sp::blocks::WriteStructure write2;

 // // Get the lowest free energy
 // sp::blocks::LowestFreeEnergy lowestE;

 // // Put it all together
 // *config.searchStartBlock |= niggli |= goPressure |= go |= remDuplicates |= sg |= write1 |= barrier |= write2 |= lowestE;

 // sp::SpStartBlock * masterPipe;
 // if(config.sweepMode)
 // {
 //   // Configure parameter sweep pipeline
 //   config.paramSweepBlock.reset(new sp::blocks::PotentialParamSweep(config.paramRange, *config.searchStartBlock));
 //   masterPipe = config.paramSweepBlock.get();
 // }
 // else
 //   masterPipe = config.searchStartBlock.get();

 // if(config.stoichSpecies.size() == 2)
 // {
 //   // Do a stoichiometry sweep
 //   config.stoichSearchBlock.reset(new sp::blocks::StoichiometrySearch(
 //     config.stoichSpecies[0],
 //     config.stoichSpecies[1],
 //     config.stoichMaxAtoms,
 //     *masterPipe
 //   ));
 //   masterPipe = config.stoichSearchBlock.get();
 // }

 // // Finally initialise and start whichever pipeline is the master
 // runner->run(*masterPipe);
}

int processCommandLineArgs(InputOptions & in, const int argc, char * argv[])
{
  const ::std::string exeName(argv[0]);

  try
  {
    po::options_description general("ssearch\nUsage: " + exeName + " [options] inpue_file...\nOptions");
    general.add_options()
      ("help", "Show help message")
      ("num,n", po::value<unsigned int>(&in.numRandomStructures)->default_value(100), "Number of random starting structures")
      ("pot,u", po::value < ::std::string>(&in.potential)->default_value("lj"), "The potential to use (possible values: lj)")
      ("input,i", po::value< ::std::string>(&in.structurePath), "The input structure")
    ;

    po::options_description lennardJones("Lennard-Jones options (when --pot lj is used)");
    lennardJones.add_options()
      ("species,s", po::value< ::std::vector< ::std::string> >(&in.potSpecies)->multitoken()_ADD_REQUIRED_, "List of species the potential applies to")
      ("params,p", po::value< ::std::vector< ::std::string> >(&in.potParams)->multitoken()_ADD_REQUIRED_, "Potential parameters, must be in quotes: eAA eAB eBB sAA sAB sBB beta [+/-1]")
      ("comb,c", po::value< ::std::string>(&in.potCombiningRule)->default_value("none"), "Off-diagonal combining rule to use")
      ("opt-press", po::value<double>(&in.optimisationPressure)->default_value(0.01), "Pressure used during initial optimisation step to bring atoms together")
      ("cutoff", po::value<double>(&in.potCutoff)->default_value(DEFAULT_POT_CUTOFF), "Potential cutoff as multiple of sigma_ij")
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

  // Everything went fine
  return 0;
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


int parseInput(RunConfiguration & config, const InputOptions & in)
{
  namespace sslib_yaml_kw = ssf::sslib_yaml_keywords;

  //config.inputType = InputType::UNKNOWN;
  //try
  //{
  //  const YAML::Node loadedNode = YAML::LoadFile(in.structurePath);
  //  
  //  ssf::SsLibFactoryYaml factory(config.speciesDb);

  //  if(loadedNode[yaml_kw::STOICHIOMETRY_SEARCH])
  //  {
  //    const YAML::Node & stoichNode = loadedNode[yaml_kw::STOICHIOMETRY_SEARCH];
  //    
  //    if(!requiredNode(stoichNode, yaml_kw::STOICHIOMETRY_SEARCH__MAX_ATOMS))
  //      return 1;
  //    if(!requiredNode(stoichNode, yaml_kw::STOICHIOMETRY_SEARCH__SPECIES))
  //      return 1;

  //    config.stoichMaxAtoms = stoichNode[yaml_kw::STOICHIOMETRY_SEARCH__MAX_ATOMS].as<unsigned int>();
  //    config.stoichSpecies =
  //      getSpeciesFromString(
  //        stoichNode[yaml_kw::STOICHIOMETRY_SEARCH__SPECIES].as< ::std::string>(),
  //        config.speciesDb
  //      );
  //  }

  //  if(loadedNode[sslib_yaml_kw::RANDOM_STRUCTURE])
  //  {
  //    config.inputType = InputType::RANDOM_STRUCTURES;
  //    ssbc::IStructureGeneratorPtr generator = factory.createStructureGenerator(loadedNode);

  //    // Random structure
  //    config.searchStartBlock.reset(new sp::blocks::RandomStructure(
  //      in.numRandomStructures,
  //      generator)
  //    );
  //  }
  //  else if(loadedNode[yaml_kw::SEED_STRUCTURES])
  //  {
  //    config.inputType = InputType::SEED_STRUCTURES;
  //    const ::std::string seedStructures = loadedNode[yaml_kw::SEED_STRUCTURES].as< ::std::string>();

  //    // Seed structures
  //    config.searchStartBlock.reset(new sp::blocks::LoadSeedStructures(
  //      seedStructures,
  //      false)
  //    );
  //  }
  //  else
  //  {
  //    ::std::cerr << "No input structure type found in input file\n";
  //    return 1;
  //  }
  //}
  //catch(const ssf::FactoryError & e)
  //{
  //  ::std::cerr << ::boost::diagnostic_information(e) << ::std::endl;
  //  return 1;
  //}

  //// Do potential parameters ////////////////////////////////
  //const int result = processPotParams(config.paramRange, config.betaDiagonal, in);
  //if(result != 0)
  //  return result;

  //// Turn on sweep mode if any of the parameters have a range (i.e. not single value)
  //config.sweepMode = false;
  //for(size_t i = 0; i < config.paramRange.nSteps.n_rows; ++i)
  //{
  //  config.sweepMode |= config.paramRange.nSteps(i) != 1;
  //}

  //// Do atom species ///////////////////////////////////////
  //if(in.potSpecies.size() != 2)
  //{
  //  ::std::cerr << "There must be 2 potential species specified\n";
  //  return 1;
  //}

  //config.potentialSpecies[0] = config.speciesDb.getIdFromSymbol(in.potSpecies[0]);
  //config.potentialSpecies[1] = config.speciesDb.getIdFromSymbol(in.potSpecies[1]);
  //config.paramRange.from(6) = config.potentialSpecies[0].ordinal();
  //config.paramRange.from(7) = config.potentialSpecies[1].ordinal();

  return 0;
}

bool requiredNode(const YAML::Node & parent, const ::std::string & childName)
{
  if(!parent[childName])
  {
    ::std::cerr << "Couldn't find required node: " << childName;
    return false;
  }
  return true;
}

::std::vector<ssc::AtomSpeciesId> getSpeciesFromString(
  const ::std::string & speciesString,
  const ssc::AtomSpeciesDatabase & speciesDb
)
{
  typedef boost::tokenizer<boost::char_separator<char> > Tok;
  const boost::char_separator<char> tokSep(" ");

  ::std::vector<ssc::AtomSpeciesId> species;

  Tok tok(speciesString, tokSep);

  ssc::AtomSpeciesId::Value id;
  BOOST_FOREACH(const ::std::string & symbol, tok)
  {
     id = speciesDb.getIdFromSymbol(symbol);
     if(id != ssc::AtomSpeciesId::DUMMY)
       species.push_back(id);
  }

  return species;
}
