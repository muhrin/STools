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

#include <armadillo>

#include <yaml-cpp/yaml.h>

// From SSTbx
#include <common/AtomSpeciesDatabase.h>

#include <pipelib/pipelib.h>

// From StructurePipe

// Local
#include "utility/PipeDataInitialisation.h"
#include "factory/StFactory.h"
#include "factory/YamlSchema.h"

// MACROS ////////////////////////////////////

// NAMESPACES ////////////////////////////////
namespace sp = ::spipe;
namespace spu = sp::utility;
namespace ssc   = ::sstbx::common;
namespace ssu   = ::sstbx::utility;
namespace ssys  = ::sstbx::yaml_schema;

// CLASSES //////////////////////////////////
struct InputOptions
{
  unsigned int      numRandomStructures;
  ::std::string     paramsFile;
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

  ssys::SchemaParse parse;
  YAML::Node buildNode;
  try
  {
    buildNode = YAML::LoadFile(in.paramsFile);
  }
  catch(const YAML::Exception & e)
  {
    ::std::cout << e.what();
    return 1;
  }
  stools::factory::Build buildSchema;
  ssu::HeterogeneousMap buildOptions;
  buildSchema.nodeToValue(parse, buildOptions, buildNode, true);

  if(parse.hasErrors())
  {
    ::std::cout << "Found errors:\n";
    parse.printErrors();
    return 1;
  }

  ssc::AtomSpeciesDatabase speciesDb;

  typedef ::sstbx::UniquePtr< ::spipe::SpPipe>::Type PipePtr;
  ::stools::factory::Factory factory(speciesDb);

  PipePtr pipe;
  factory.createSearchPipe(pipe, buildOptions);

  // Now run the pipe
  typedef sp::SpSingleThreadedEngine Engine;
  typedef Engine::RunnerPtr RunnerPtr;

  Engine pipeEngine;
  RunnerPtr runner = spu::generateRunnerInitDefault(pipeEngine);
  runner->run(*pipe);

  return 0;
}

int processCommandLineArgs(InputOptions & in, const int argc, char * argv[])
{
  namespace po = ::boost::program_options;

  const ::std::string exeName(argv[0]);

  try
  {
    po::options_description general("sbuild\nUsage: " + exeName + " [options] inpue_file...\nOptions");
    general.add_options()
      ("help", "Show help message")
      ("num,n", po::value<unsigned int>(&in.numRandomStructures)->default_value(1), "Number of random starting structures")
      ("input,i", po::value< ::std::string>(&in.paramsFile), "The file containing the structure configuration")
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
