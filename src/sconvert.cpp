/*
 * sconvert.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "STools.h"

#include <boost/program_options.hpp>


#include <spl/common/AtomSpeciesDatabase.h>
#include <spl/common/Structure.h>
#include <spl/io/StructureReadWriteManager.h>

// From StructurePipe
#include <utility/PipeDataInitialisation.h>

// Local //
#include "utility/BoostCapabilities.h"

// MACROS ////////////////////////////////////

// NAMESPACES ////////////////////////////////
namespace fs    = ::boost::filesystem;
namespace po    = ::boost::program_options;
namespace sp    = ::spipe;
namespace ssc   = ::spl::common;
namespace ssio  = ::spl::io;
namespace ssu   = ::spl::utility;

typedef ::std::vector< ::std::string> StringsVector;

struct InputOptions
{
  StringsVector inputOutputFiles;
};

int processCommandLineArgs(InputOptions & in, const int argc, char * argv[]);

// CONSTANTS /////////////////////////////////


int main(const int argc, char * argv[])
{
  // Program options
  InputOptions in;

  int result = processCommandLineArgs(in, argc, argv);
  if(result != 0)
    return result;

  ssio::ResourceLocator fileIn, fileOut;
  if(!fileIn.set(in.inputOutputFiles[0]))
    ::std::cerr << "Input file not valid" << ::std::endl;
  if(!fileOut.set(in.inputOutputFiles[1]))
    ::std::cerr << "Output file not valid" << ::std::endl;

  ssio::StructureReadWriteManager rwMan;
  sp::utility::initStructureRwManDefault(rwMan);

  ssio::StructuresContainer structures;
  if(rwMan.readStructures(structures, fileIn) == 0)
  {
    ::std::cerr << "Error: Failed to read any structures." << ::std::endl;
    return 1;
  }

  BOOST_FOREACH(ssc::Structure & structure, structures)
  {
    if(!rwMan.writeStructure(structure, fileOut))
      ::std::cerr << "Error: Failed to write structure to " << fileOut << ::std::endl;
  }

  return 0;
}

int processCommandLineArgs(InputOptions & in, const int argc, char * argv[])
{
  const ::std::string exeName(argv[0]);

  in.inputOutputFiles.resize(2);
  try
  {
    po::options_description general("sconvert\nUsage: " + exeName + " [input-options] inpue-file(s) [output-options] output-file(s)\nOptions");
    general.add_options()
      ("help", "Show help message")
      ("input,i", po::value<StringsVector>(&in.inputOutputFiles)_ADD_REQUIRED_, "Input and output filenames")
    ;

    po::positional_options_description p;
    p.add("input", 2);

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


