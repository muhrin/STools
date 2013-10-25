/*
 * sgroup.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include <list>
#include <string>

#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <boost/program_options.hpp>

// From Pipelib //

//
#include <spl/analysis/SpaceGroup.h>
#include <spl/common/Structure.h>
#include <spl/common/Types.h>
#include <spl/io/ResourceLocator.h>
#include <spl/io/StructureReadWriteManager.h>

// From StructurePipe
#include <utility/PipeDataInitialisation.h>

// My includes //

// NAMESPACES ////////////////////////////////
namespace fs = ::boost::filesystem;
namespace po = ::boost::program_options;
namespace sp = ::spipe;
namespace ssc = ::spl::common;
namespace ssio = ::spl::io;
namespace ssa = ::spl::analysis;

struct InputOptions
{
  double precision;
  ::std::vector< ::std::string> inputFiles;
  bool printSgNumber;
};

int
main(const int argc, char * argv[])
{
  const ::std::string exeName(argv[0]);

  // Input options
  InputOptions in;

  try
  {
    po::options_description desc(
        "Usage: " + exeName + " [options] files...\nOptions:");
    desc.add_options()("help", "Show help message")("prec,p",
        po::value< double>(&in.precision)->default_value(
            ssa::space_group::DEFAULT_PRECISION),
        "Set space group identifier precision")("input-file",
        po::value< ::std::vector< ::std::string> >(&in.inputFiles),
        "input file(s)")("num,n",
        po::value< bool>(&in.printSgNumber)->default_value(false)->zero_tokens(),
        "print space group number");

    po::positional_options_description p;
    p.add("input-file", -1);

    po::variables_map vm;
    po::store(
        po::command_line_parser(argc, argv).options(desc).positional(p).run(),
        vm);

    // Deal with help first, otherwise missing required parameters will cause exception
    if(vm.count("help"))
    {
      ::std::cout << desc << ::std::endl;
      return 1;
    }

    po::notify(vm);
  }
  catch(std::exception& e)
  {
    ::std::cout << e.what() << ::std::endl;
    return 1;
  }

  ssio::StructureReadWriteManager rwMan;
  sp::utility::initStructureRwManDefault(rwMan);

  ssio::StructuresContainer loadedStructures;
  ssio::ResourceLocator structureLocator;
  ssa::space_group::SpacegroupInfo sgInfo;

  BOOST_FOREACH(const ::std::string & inputFile, in.inputFiles)
  {
    if(!structureLocator.set(inputFile))
    {
      ::std::cerr << "Invalid structure path " << inputFile << ". Skipping."
          << ::std::endl;
      continue;
    }
    if(!fs::exists(structureLocator.path()))
    {
      ::std::cerr << "File " << inputFile << " does not exist.  Skipping."
          << ::std::endl;
      continue;
    }

    if(rwMan.readStructures(loadedStructures, structureLocator) > 0)
    {
      BOOST_FOREACH(const ssc::Structure & structure, loadedStructures)
      {
        if(ssa::space_group::getSpacegroupInfo(sgInfo, structure, in.precision))
        {
          if(in.printSgNumber)
            ::std::cout << sgInfo.number << ::std::endl;
          else
            ::std::cout << sgInfo.iucSymbol << ::std::endl;
        }
      }
      loadedStructures.clear();
    }
  }

}
