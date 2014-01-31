/*
 * StructurePipe.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include <StructurePipe.h>

#include <boost/program_options.hpp>

#include <yaml-cpp/yaml.h>

#include <schemer/Schemer.h>

#include <spl/io/BoostFilesystem.h>

#include <pipelib/pipelib.h>

// Local includes
#include "build/PipeBuilder.h"
#include "factory/PipeEngineFactory.h"
#include "factory/PipeEngineSchema.h"
#include "utility/Initialisation.h"

// MACROS ////////////////////////////////////

// NAMESPACES ////////////////////////////////
using namespace spipe;

// CLASSES //////////////////////////////////
struct InputOptions
{
  std::string input;
  std::vector< std::string> additionalOptions;
};

struct PipeOptions
{
  std::string rngSeed;
  spipe::factory::Engine engine;
  std::string run;
};

SCHEMER_MAP(PipeOptionsType, PipeOptions)
{
  element("rngSeed", &PipeOptions::rngSeed)->defaultValue("time");
  element("engine", &PipeOptions::engine);
  element("run", &PipeOptions::run);
}

// FUNCTIONS ////////////////////////////////
int
processCommandLineArgs(InputOptions & in, const int argc, char * argv[]);

int
main(const int argc, char * argv[])
{
  // Program options
  InputOptions in;

  int result = processCommandLineArgs(in, argc, argv);
  if(result != 0)
    return result;

  if(!boost::filesystem::exists(in.input))
  {
    std::cerr << "Error: file " << in.input << " does not exist\n";
    return 1;
  }

  YAML::Node pipeNode;
  result = utility::parseYaml(pipeNode, in.input);
  if(result != 0)
    return result;

  // Add any additional options specified at the command line
  if(!utility::insertScalarValues(pipeNode, in.additionalOptions))
    return 1;

  PipeOptions pipeOptions;
  schemer::ParseLog log;
  if(!schemer::parse(pipeNode, &pipeOptions, &log))
  {
    log.printErrors(&std::cerr);
    return 1;
  }

  utility::seedRandomNumberGenerator(pipeOptions.rngSeed);

  spipe::BlockHandle startBlock = spipe::build::buildPipe(pipeNode);
  if(!startBlock)
  {
    std::cerr << "Error: Failed to create pipe" << "\n";
    return 1;
  }

  static const factory::PipeEngineFactory ENGINE_FACTORY =
      spipe::factory::PipeEngineFactory();

  factory::PipeEngineFactory::EnginePtr engine = ENGINE_FACTORY.createEngine(
      pipeOptions.engine);
  if(!engine.get())
  {
    std::cerr << "Error: Failed to create pipe engine" << std::endl;
    return 1;
  }
  engine->globalData().setSeedName(spl::io::stemString(in.input));
  engine->run(startBlock);

  return 0;
}

int
processCommandLineArgs(InputOptions & in, const int argc, char * argv[])
{
  namespace po = boost::program_options;

  const std::string exeName(argv[0]);

  try
  {
    po::options_description general(
        "spipe\nUsage: " + exeName + " [options] inpue_file...\nOptions");
    general.add_options()("help", "Show help message")("input,i",
        po::value< std::string>(&in.input), "The structure pipe file")(
        "define,D",
        po::value< std::vector< std::string> >(&in.additionalOptions)->composing(),
        "Define program options on the command line as if they had been included in the input file");

    po::positional_options_description p;
    p.add("input", 1);

    po::options_description cmdLineOptions;
    cmdLineOptions.add(general);

    po::variables_map vm;
    po::store(
        po::command_line_parser(argc, argv).options(cmdLineOptions).positional(
            p).run(), vm);

    // Deal with help first, otherwise missing required parameters will cause exception on vm.notify
    if(vm.count("help"))
    {
      std::cout << cmdLineOptions << std::endl;
      return 1;
    }

    po::notify(vm);
  }
  catch(std::exception& e)
  {
    std::cout << e.what() << "\n";
    return 1;
  }

  // Everything went fine
  return 0;
}
