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


#include <spl/common/AtomSpeciesDatabase.h>
#include <spl/io/BoostFilesystem.h>

#include <pipelib/pipelib.h>

// From StructurePipe
#include <factory/PipeEngine.h>
#include <factory/PipeFactory.h>

// Local
#include "utility/PipeDataInitialisation.h"
#include "input/OptionsParsing.h"
#include "factory/YamlSchema.h"

// MACROS ////////////////////////////////////
//#define STOOLS_DEBUG

#ifdef STOOLS_DEBUG
#  include <fenv.h>
#endif

// NAMESPACES ////////////////////////////////
using namespace ::stools;

namespace fs = ::boost::filesystem;
namespace sp = ::spipe;
namespace spu = sp::utility;
namespace ssu   = ::spl::utility;
namespace ssys  = ::spl::yaml_schema;

// CLASSES //////////////////////////////////
struct InputOptions
{
  ::std::string inputOptionsFile;
  ::std::vector< ::std::string> additionalOptions;
};

// CONSTANTS /////////////////////////////////

// FUNCTIONS ////////////////////////////////
int processCommandLineArgs(InputOptions & in, const int argc, char * argv[]);

int main(const int argc, char * argv[])
{
#ifdef STOOLS_DEBUG
  feenableexcept(FE_DIVBYZERO|FE_INVALID|FE_OVERFLOW);
#endif

  // Program options
  InputOptions in;

  int result = processCommandLineArgs(in, argc, argv);
  if(result != 0)
    return result;

  if(!fs::exists(in.inputOptionsFile))
  {
    ::std::cerr << "Couldn't find input file " << in.inputOptionsFile << ::std::endl;
    return 1;
  }

  // Read the yaml options
  YAML::Node searchNode;
  result = input::parseYaml(searchNode, in.inputOptionsFile);
  if(result != 0)
    return result;

  // Add any additional options specified at the command line
  if(!input::insertScalarValues(searchNode, in.additionalOptions))
    return false;
  
  // Parse the yaml
  ssys::SchemaParse parse;
  factory::Search searchSchema;
  ssu::HeterogeneousMap schemaOptions;
  searchSchema.nodeToValue(parse, schemaOptions, searchNode, true);
  if(parse.hasErrors())
  {
    parse.printErrors();
    return 1;
  }
  ::stools::input::seedRandomNumberGenerator(schemaOptions);

  // Create the pipe the run the search
  factory::PipeEnginePtr engine = factory::createPipeEngine(schemaOptions);
  if(!engine.get())
  {
    ::std::cerr << "Error: Failed to create pipe engine" << ::std::endl;
    return 1;
  }

  engine->globalData().setSeedName(::spl::io::stemString(in.inputOptionsFile));
  factory::PipeFactory factory(engine->globalData().getSpeciesDatabase());

  sp::BlockHandle pipe = factory.createSearchPipeExtended(schemaOptions);
  if(!pipe)
  {
    ::std::cerr << "Error: Failed to create search pipe" << ::std::endl;
    return 1;
  }

  engine->run(pipe);

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
      ("input,i", po::value< ::std::string>(&in.inputOptionsFile), "The input options file")
      ("define,D", po::value< ::std::vector< ::std::string> >(&in.additionalOptions)->composing(),
      "Define program options on the command line as if they had been included in the input file")
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


