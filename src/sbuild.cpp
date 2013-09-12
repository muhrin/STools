/*
 * ssearch.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "STools.h"

#include <iostream>
#include <string>

#include <boost/program_options.hpp>

#include <yaml-cpp/yaml.h>

#include <pipelib/pipelib.h>

// From StructurePipe
#include <factory/PipeEngine.h>
#include <factory/StFactory.h>

// Local
#include "factory/YamlSchema.h"
#include "input/OptionsParsing.h"
#include "utility/BoostCapabilities.h"
#include "utility/PipeDataInitialisation.h"

// MACROS ////////////////////////////////////

// NAMESPACES ////////////////////////////////
namespace sp = ::spipe;
namespace splys = ::spl::yaml_schema;
namespace splu = ::spl::utility;
namespace factory = ::stools::factory;

// CLASSES //////////////////////////////////
struct InputOptions
{
  unsigned int numRandomStructures;
  ::std::string inputOptionsFile;
};

// CONSTANTS /////////////////////////////////

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

  // Load up the yaml options
  YAML::Node buildNode;
  result = ::stools::input::parseYaml(buildNode, in.inputOptionsFile);
  if(result != 0)
    return result;

  splys::SchemaParse parse;
  stools::factory::Build buildSchema;
  splu::HeterogeneousMap buildOptions;
  buildSchema.nodeToValue(parse, buildOptions, buildNode, true);
  if(parse.hasErrors())
  {
    ::std::cout << "Found errors:\n";
    parse.printErrors();
    return 1;
  }
  ::stools::input::seedRandomNumberGenerator(buildOptions);

  // Create the pipe engine to drive the pipe
  factory::PipeEnginePtr engine = factory::createPipeEngine(buildOptions);
  if(!engine.get())
  {
    ::std::cerr << "Error: Failed to create pipe engine" << ::std::endl;
    return 1;
  }

  engine->globalData().setSeedName(::spl::io::stemString(in.inputOptionsFile));

  ::stools::factory::Factory factory(engine->globalData().getSpeciesDatabase());
  sp::BlockHandle pipe = factory.createBuildPipe(buildOptions);
  if(!pipe)
  {
    ::std::cerr << "Failed to create build pipe" << ::std::endl;
    return 1;
  }

  engine->run(pipe);

  return 0;
}

int
processCommandLineArgs(InputOptions & in, const int argc, char * argv[])
{
  namespace po = ::boost::program_options;

  const ::std::string exeName(argv[0]);

  try
  {
    po::options_description general(
        "sbuild\nUsage: " + exeName + " [options] inpue_file...\nOptions");
    general.add_options()("help", "Show help message")("num,n",
        po::value< unsigned int>(&in.numRandomStructures)->default_value(1),
        "Number of random starting structures")("input,i",
        po::value< ::std::string>(&in.inputOptionsFile)_ADD_REQUIRED_,
        "The file containing the structure configuration");

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

  // Everything went fine
  return 0;
}
