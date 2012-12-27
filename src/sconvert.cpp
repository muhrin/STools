/*
 * sconvert.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "STools.h"

#include <boost/program_options.hpp>

// From SSTbx
#include <common/AtomSpeciesDatabase.h>
#include <common/Structure.h>
#include <io/StructureReadWriteManager.h>


// From StructurePipe
#include <utility/PipeDataInitialisation.h>

// MACROS ////////////////////////////////////

// NAMESPACES ////////////////////////////////
namespace fs    = ::boost::filesystem;
namespace po    = ::boost::program_options;
namespace spu   = ::spipe::utility;
namespace ssc   = ::sstbx::common;
namespace ssio  = ::sstbx::io;
namespace ssu   = ::sstbx::utility;

typedef ::std::vector< ::std::string> StringsVector;

struct InputOptions
{
  StringsVector inputOutputFiles;
};

int processCommandLineArgs(InputOptions & in, const int argc, const char * const argv[]);

// CONSTANTS /////////////////////////////////


int main(const int argc, const char * const argv[])
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

  ssc::AtomSpeciesDatabase speciesDb;

  ssio::StructureReadWriteManager rwMan;
  spu::initStructureRwManDefault(rwMan);

  ssio::StructuresContainer structures;
  if(rwMan.readStructures(structures, fileIn, speciesDb) == 0)
  {
    ::std::cerr << "Failed to ready any structures." << ::std::endl;
    return 1;
  }

  BOOST_FOREACH(ssc::Structure & structure, structures)
  {
    rwMan.writeStructure(structure, fileOut, speciesDb);
  }

  return 0;
}

int processCommandLineArgs(InputOptions & in, const int argc, const char * const argv[])
{
  const ::std::string exeName(argv[0]);

  try
  {
    po::options_description general("STools\nUsage: " + exeName + " [options] inpue_file output_file\nOptions");
    general.add_options()
      ("help", "Show help message")
      ("input,i", po::value<StringsVector>(&in.inputOutputFiles), "Input and output filenames")
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


