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

#include <spl/common/AtomSpeciesDatabase.h>

#include <pipelib/pipelib.h>

#include "build/PipeBuilder.h"

// MACROS ////////////////////////////////////

// NAMESPACES ////////////////////////////////

// CLASSES //////////////////////////////////
struct InputOptions
{
  ::std::string input;
};

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

  YAML::Node pipeNode;
  try
  {
    pipeNode = YAML::LoadFile(in.input);
  }
  catch(const YAML::Exception & /*e*/)
  {
    std::cerr << "Error: Failed reading yaml in " + in.input << "\n";
    return 1;
  }
  spl::common::AtomSpeciesDatabase speciesDb;
  spipe::BlockHandle startBlock = spipe::build::buildPipe(pipeNode, speciesDb);

  spipe::SerialEngine engine;
  engine.run(startBlock);

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
        "spipe\nUsage: " + exeName + " [options] inpue_file...\nOptions");
    general.add_options()("help", "Show help message")("input,i",
        po::value< std::string>(&in.input), "The structure pipe file");

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
