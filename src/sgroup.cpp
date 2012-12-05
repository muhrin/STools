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


// From SSLib //
#include <analysis/SpaceGroup.h>
#include <common/AtomSpeciesDatabase.h>
#include <common/Structure.h>
#include <common/Types.h>
#include <io/ResReaderWriter.h>

// My includes //

// NAMESPACES ////////////////////////////////
namespace fs = ::boost::filesystem;
namespace po = ::boost::program_options;
namespace ssc = ::sstbx::common;
namespace ssio = ::sstbx::io;
namespace ssa = ::sstbx::analysis;

struct InputOptions
{
  double precision;
  ::std::vector< ::std::string> inputFiles;
  bool printSgNumber;
};

int main(const int argc, char * argv[])
{
  const ::std::string exeName(argv[0]);

  // Input options
  InputOptions in;

  try
  {
    po::options_description desc("Usage: " + exeName + " [options] files...\nOptions:");
    desc.add_options()
      ("help", "Show help message")
      ("prec,p", po::value<double>(&in.precision)->default_value(ssa::space_group::DEFAULT_PRECISION), "Set space group identifier precision")
      ("input-file", po::value< ::std::vector< ::std::string> >(&in.inputFiles), "input file(s)")
      ("num,n", po::value<bool>(&in.printSgNumber)->default_value(false), "print spacegroup number")
    ;

    po::positional_options_description p;
    p.add("input-file", -1);

    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).options(desc).positional(p).run(), vm);

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


  ssc::AtomSpeciesDatabase speciesDb;
  ssio::ResReaderWriter resReader;
  ssc::StructurePtr structure;
  ssa::space_group::SpacegroupInfo sgInfo;

  BOOST_FOREACH(::std::string & pathString, in.inputFiles)
  {
    fs::path strPath(pathString);
    if(!fs::exists(strPath))
    {
      ::std::cerr << "File " << strPath << " does not exist.  Skipping" << ::std::endl;
      continue;
    }

    structure = resReader.readStructure(strPath, speciesDb);
    if(structure.get() && ssa::space_group::getSpacegroupInfo(sgInfo, *structure.get(), in.precision))
    {
      if(in.printSgNumber)
        ::std::cout << sgInfo.number << ::std::endl;
      else
        ::std::cout << sgInfo.iucSymbol << ::std::endl;
    }
  }


}
