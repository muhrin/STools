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
#include <boost/ptr_container/ptr_map.hpp>

#include <armadillo>

// From Pipelib //


// From SSLib //
#include <common/AtomSpeciesDatabase.h>
#include <common/Structure.h>
#include <common/Types.h>
#include <common/UnitCell.h>
#include <io/ResReaderWriter.h>
#include <utility/DistanceMatrixComparator.h>
#include <utility/SortedDistanceComparator.h>
#include <utility/SortedDistanceComparatorEx.h>
#include <utility/IBufferedComparator.h>

// My includes //


// NAMESPACES ////////////////////////////////
namespace fs = ::boost::filesystem;
namespace po = ::boost::program_options;
namespace ssu = ::sstbx::utility;
namespace ssc = ::sstbx::common;
namespace ssio = ::sstbx::io;

struct InputOptions
{
  double tolerance;
  ::std::vector< ::std::string> inputFiles;
  ::std::string comparator;
  bool printFull;
  unsigned int maxAtoms;
};

int main(const int argc, char * argv[])
{
  typedef ::boost::shared_ptr<ssc::Structure> SharedStructurePtr;
  typedef ::std::pair<fs::path, SharedStructurePtr> PathStructurePair;

  const ::std::string exeName(argv[0]);

  // Input options
  InputOptions in;

  try
  {
    po::options_description desc("Usage: " + exeName + " [options] files...\nOptions:");
    desc.add_options()
      ("help", "Show help message")
      ("tol,t", po::value<double>(&in.tolerance)->default_value(0.01), "Set comparator tolerance")
      ("input-file", po::value< ::std::vector< ::std::string> >(&in.inputFiles)->required(), "input file(s)")
      ("full", po::value<bool>(&in.printFull)->default_value(false)->zero_tokens(), "Print full matrix, not just lower triangular")
      ("maxatoms", po::value<unsigned int>(&in.maxAtoms)->default_value(12), "The maximum number of atoms before switching to fast comparison method.")
      ("comp,c", po::value< ::std::string>(&in.comparator)->default_value("sd"), "The comparator to use: sd = sorted distance, sdex = sorted distance extended, dm = distance matrix")
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
    ::std::cout << e.what() << "\n";
    return 1;
  }

  ::boost::scoped_ptr<ssu::IStructureComparator> comp;

  if(in.comparator == "sd")
  {
    comp.reset(new ssu::SortedDistanceComparator());
  }
  else if(in.comparator == "sdex")
  {
    comp.reset(new ssu::SortedDistanceComparatorEx());
  }
  else if(in.comparator == "dm")
  {
    comp.reset(new ssu::DistanceMatrixComparator(in.maxAtoms));
  }
  else
  {
    ::std::cout << "Error: unrecognised comparator - " << in.comparator << ::std::endl;
    return 1;
  }
  
  ssc::AtomSpeciesDatabase speciesDb;
  ssio::ResReaderWriter resReader;
  ::std::vector<PathStructurePair> structures;
  ssc::StructurePtr str;

  BOOST_FOREACH(::std::string & pathString, in.inputFiles)
  {
    fs::path strPath(pathString);
    if(!fs::exists(strPath))
    {
      ::std::cerr << "File " << strPath << " does not exist.  Skipping" << ::std::endl;
      continue;
    }

    str = resReader.readStructure(strPath, speciesDb);
    if(str.get())
      structures.push_back(PathStructurePair(strPath, SharedStructurePtr(str.release())));
  }

  if(structures.size() < 2)
  {
    ::std::cout << "Not enough structures to compare." << ::std::endl;
    return 1;
  }

  const size_t numStructures = structures.size();
  ::boost::shared_ptr<ssu::IBufferedComparator> comparator = comp->generateBuffered();

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

  size_t maxJ;
  for(size_t i = 0; i < numStructures; ++i)
  {
    maxJ = in.printFull ? numStructures: i;
    for(size_t j = 0; j < maxJ; ++j)
    {
      ::std::cout << diffs(i, j) << " ";
    }
    ::std::cout << ::std::endl;
  }


	return 0;
}

