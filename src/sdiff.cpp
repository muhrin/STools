/*
 * sdiff.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "StructurePipe.h"

#include <iostream>
#include <string>

#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/program_options.hpp>

#include <armadillo>

// From Pipelib //


// From SSLib //
#include <common/Structure.h>
#include <common/Types.h>
#include <common/UnitCell.h>
#include <io/ResReaderWriter.h>
#include <utility/DistanceMatrixComparator.h>
#include <utility/IBufferedComparator.h>

// My includes //


// NAMESPACES ////////////////////////////////
namespace fs = ::boost::filesystem;
namespace po = ::boost::program_options;
namespace ssu = ::sstbx::utility;
namespace ssc = ::sstbx::common;

int main(const int argc, char * argv[])
{
  typedef ::std::pair<fs::path, ssc::StructurePtr> PathStructurePair;

  // Input options
  double tolerance;
  ::std::vector< ::std::string> inputFiles;
  bool printFull;

  try
  {
    po::options_description desc("Allowed options");
    desc.add_options()
      ("help", "Show help message")
      ("tol,t", po::value<double>(&tolerance)->default_value(0.01), "Set comparator tolerance")
      ("input-file", po::value< ::std::vector< ::std::string> >(&inputFiles), "input file(s)")
      ("full", po::value<bool>(&printFull)->default_value(false)->zero_tokens(), "Print full matrix, not just upper triangular")
    ;

    po::positional_options_description p;
    p.add("input-file", -1);

    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).options(desc).positional(p).run(), vm);
    po::notify(vm);

    if(vm.count("help"))
    {
      ::std::cout << desc << ::std::endl;
      return 1;
    }
  }
  catch(std::exception& e)
  {
    ::std::cout << e.what() << "\n";
    return 1;
  }   


  sstbx::io::ResReaderWriter resReader;
  ::std::vector<PathStructurePair> structures;

  BOOST_FOREACH(::std::string & pathString, inputFiles)
  {
    fs::path strPath(pathString);
    if(!fs::exists(strPath))
    {
      ::std::cerr << "File " << strPath << " does not exist.  Skipping" << ::std::endl;
      continue;
    }

    PathStructurePair pair(strPath, ssc::StructurePtr(new ssc::Structure()));
    resReader.readStructure(*pair.second.get(), pair.first);

    structures.push_back(pair);
  }

  const size_t numStructures = structures.size();
  ssu::DistanceMatrixComparator comp(0);
  ::boost::shared_ptr<ssu::IBufferedComparator> comparator = comp.generateBuffered();

  ::arma::mat diffs(numStructures, numStructures);
  diffs.diag().fill(0.0);

  for(size_t i = 0; i < numStructures - 1; ++i)
  {
    for(size_t j = i + 1; j < numStructures; ++j)
    {
      diffs(i, j) = comparator->compareStructures(
        *structures[i].second.get(),
        *structures[j].second.get());
    }
  }
  diffs = ::arma::symmatu(diffs);

  size_t minJ;
  for(size_t i = 0; i < numStructures; ++i)
  {
    minJ = printFull ? 0 : i + 1;
    for(size_t j = minJ; j < numStructures; ++j)
    {
      ::std::cout << diffs(i, j) << " ";
    }
    ::std::cout << ::std::endl;
  }


	return 0;
}

