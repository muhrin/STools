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

// From SSTbx
#include <common/AtomSpeciesDatabase.h>
#include <io/BoostFilesystem.h>

#include <pipelib/pipelib.h>

// From StructurePipe

// Local
#include "utility/PipeDataInitialisation.h"
#include "input/OptionsParsing.h"
#include "factory/StFactory.h"
#include "factory/YamlSchema.h"

// MACROS ////////////////////////////////////

// NAMESPACES ////////////////////////////////
namespace fs = ::boost::filesystem;
namespace sp = ::spipe;
namespace spu = sp::utility;
namespace ssc   = ::sstbx::common;
namespace ssu   = ::sstbx::utility;
namespace ssys  = ::sstbx::yaml_schema;

// CLASSES //////////////////////////////////
struct InputOptions
{
  ::std::string inputOptionsFile;
  unsigned int numRandomStructures;
  double optimisationPressure;
};

// CONSTANTS /////////////////////////////////

// FUNCTIONS ////////////////////////////////
int processCommandLineArgs(InputOptions & in, const int argc, char * argv[]);

int main(const int argc, char * argv[])
{
  // Program options
  InputOptions in;

  int result = processCommandLineArgs(in, argc, argv);
  if(result != 0)
    return result;

  if(!fs::exists(in.inputOptionsFile))
    return 1;

  // Load up the yaml options
  YAML::Node searchNode;
  result = ::stools::input::parseYaml(searchNode, in.inputOptionsFile);
  if(result != 0)
    return result;
  
  // Parse the yaml
  ssys::SchemaParse parse;
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
  if(!factory.createSearchPipeExtended(pipe, schemaOptions))
  {
    ::std::cerr << "Failed to create search pipe" << ::std::endl;
    return 1;
  }

  // Now run the pipe
  typedef sp::SpSingleThreadedEngine Engine;
  typedef Engine::RunnerPtr RunnerPtr;

  Engine pipeEngine;
  RunnerPtr runner = spu::generateRunnerInitDefault(pipeEngine);
  runner->memory().global().setSeedName(::sstbx::io::stemString(in.inputOptionsFile));
  runner->run(*pipe);

  return 0;
}

int processCommandLineArgs(InputOptions & in, const int argc, char * argv[])
{
  namespace po = ::boost::program_options;

  const ::std::string exeName(argv[0]);

  try
  {
    po::options_description general("ssearch\nUsage: " + exeName + " [options] inpue_file...\nOptions");
    general.add_options()
      ("help", "Show help message")
      ("num,n", po::value<unsigned int>(&in.numRandomStructures)->default_value(100), "Number of random starting structures")
      ("input,i", po::value< ::std::string>(&in.inputOptionsFile), "The input options file")
    ;

    po::positional_options_description p;
    p.add("input", 1);

    po::options_description cmdLineOptions;
    cmdLineOptions.add(general);

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

  // Everything went fine
  return 0;
}
