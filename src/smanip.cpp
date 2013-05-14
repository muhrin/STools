/*
 * smanip.cpp
 *
 *  Created on: May 14, 2013
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

// Local //
#include "utility/BoostCapabilities.h"

// MACROS ////////////////////////////////////

// NAMESPACES ////////////////////////////////
namespace fs    = ::boost::filesystem;
namespace po    = ::boost::program_options;
namespace sp    = ::spipe;
namespace ssc   = ::sstbx::common;
namespace ssio  = ::sstbx::io;
namespace ssu   = ::sstbx::utility;
namespace structure_properties = ssc::structure_properties;

typedef ::std::vector< ::std::string> StringsVector;

struct InputOptions
{
  StringsVector inputFiles;
  bool niggliReduce;
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

  ssc::AtomSpeciesDatabase speciesDb;

  ssio::StructureReadWriteManager rwMan;
  sp::utility::initStructureRwManDefault(rwMan);

  ssio::StructuresContainer structures;
  size_t totalLoaded = 0;
  BOOST_FOREACH(const ::std::string & inputFile, in.inputFiles)
  {
    ssio::ResourceLocator loc;
    if(!loc.set(inputFile))
    {
      ::std::cerr << "Invalid file locator: " << loc << ::std::endl;
      continue;
    }
    if(!fs::exists(loc.path()))
    {
      ::std::cerr << "File " << loc.path().string() << " does not exist.  Skipping" << ::std::endl;
      continue;
    }

    size_t lastLoaded = rwMan.readStructures(structures, loc, speciesDb);
    if(lastLoaded == 0)
      ::std::cerr << "Couldn't load structure(s) from " << loc.string() << ::std::endl;
  }


  BOOST_FOREACH(ssc::Structure & structure, structures)
  {
    const ssio::ResourceLocator * locator = structure.getProperty(structure_properties::io::LAST_ABS_FILE_PATH);
    if(!locator)
      continue;

    if(in.niggliReduce)
    {
      if(structure.getUnitCell())
      {
        structure.getUnitCell()->niggliReduce();
      }
    }


    if(!rwMan.writeStructure(structure, *locator, speciesDb))
      ::std::cerr << "Failed to write structure to " << *locator << ::std::endl;
  }

  return 0;
}

int processCommandLineArgs(InputOptions & in, const int argc, char * argv[])
{
  const ::std::string exeName(argv[0]);

  try
  {
    po::options_description general("smanip\nUsage: " + exeName + " [options] input-file(s)\nOptions");
    general.add_options()
      ("help", "Show help message")
      ("input,i", po::value<StringsVector>(&in.inputFiles)_ADD_REQUIRED_, "Input files")
      ("niggli", po::value<bool>(&in.niggliReduce)->zero_tokens()->default_value(false), "Niggli reduce unit cell")
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


