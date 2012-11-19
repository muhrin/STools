/*
 * sdiff.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "StructurePipe.h"

#include <functional>
#include <iostream>
#include <string>

#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/program_options.hpp>
#include <boost/ptr_container/ptr_map.hpp>
#include <boost/iterator/transform_iterator.hpp>

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
#include <utility/TransformFunctions.h>
#include <utility/UniqueStructureSet.h>

// My includes //
#include "utility/TerminalFunctions.h"


// NAMESPACES ////////////////////////////////
namespace fs = ::boost::filesystem;
namespace po = ::boost::program_options;
namespace ssu = ::sstbx::utility;
namespace ssc = ::sstbx::common;
namespace ssio = ::sstbx::io;

typedef ::boost::shared_ptr<ssc::Structure> SharedStructurePtr;
typedef ::std::pair<fs::path, SharedStructurePtr> PathStructurePair;
typedef ::std::vector<PathStructurePair> StructuresList;
typedef ::boost::shared_ptr<ssu::IBufferedComparator> ComparatorPtr;

struct InputOptions
{
  double tolerance;
  ::std::vector< ::std::string> inputFiles;
  ::std::string comparator;
  bool printFull;
  unsigned int maxAtoms;
  bool uniqueOnly;
  bool volumeAgnostic;
  bool dontUsePrimitive;
};

// FORWARD DECLARES //////////
void doUniques(const StructuresList & structures, ComparatorPtr comparator);
void doDiff(const StructuresList & structures, ComparatorPtr comparator, const InputOptions & in);

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
      ("tol,t", po::value<double>(&in.tolerance)->default_value(0.01), "Set comparator tolerance")
      ("input-file", po::value< ::std::vector< ::std::string> >(&in.inputFiles), "input file(s)")
      ("full", po::value<bool>(&in.printFull)->default_value(false)->zero_tokens(), "Print full matrix, not just lower triangular")
      ("maxatoms", po::value<unsigned int>(&in.maxAtoms)->default_value(12), "The maximum number of atoms before switching to fast comparison method.")
      ("comp,c", po::value< ::std::string>(&in.comparator)->default_value("sd"), "The comparator to use: sd = sorted distance, sdex = sorted distance extended, dm = distance matrix")
      ("agnostic,a", po::value<bool>(&in.volumeAgnostic)->default_value(false)->zero_tokens(), "Volume agnostic: volume/atom to 1 for each structure before performing comparison")
      ("no-primitive,p", po::value<bool>(&in.dontUsePrimitive)->default_value(false)->zero_tokens(), "Do not transform structures to primitive setting before comparison")
      ("unique,u", po::value<bool>(&in.uniqueOnly)->default_value(false)->zero_tokens(), "Print a list of the paths to the unique structures only from the list of input structures")
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

  // Get any input from standard in (piped)
  std::string lineInput;
  bool foundPipedInput = false;
  if(stools::utility::isStdInPipedOrFile())
  {
    while(std::cin >> lineInput)
    {
      in.inputFiles.push_back(lineInput);
      foundPipedInput = true;
    }
  }

  if(!foundPipedInput && in.inputFiles.empty())
  {
    std::cout << "No structure files given.  Supply files with piped input or as command line paramter." << std::endl;
    return 1;
  }

  ::boost::scoped_ptr<ssu::IStructureComparator> comp;

  if(in.comparator == "sd")
  {
    comp.reset(new ssu::SortedDistanceComparator(ssu::SortedDistanceComparator::DEFAULT_TOLERANCE, false, false));
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
  StructuresList structures;
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
    {
      if(!in.dontUsePrimitive)
        str->makePrimitive();

      // If we are to be volume agnostic then set the volume
      // to 1.0 per atom
      ssc::UnitCell * const unitCell = str->getUnitCell();
      if(in.volumeAgnostic && unitCell)
      {
        const double scaleFactor = str->getNumAtoms() / unitCell->getVolume();
        str->scale(scaleFactor);
      }


      structures.push_back(PathStructurePair(strPath, SharedStructurePtr(str.release())));
    }
  }

  if(structures.size() < 2)
  {
    ::std::cout << "Not enough structures to compare." << ::std::endl;
    return 1;
  }

  ComparatorPtr comparator = comp->generateBuffered();


  // Do the actual comparison based on command line options
  if(in.uniqueOnly)
  {
    doUniques(structures, comparator);
  }
  else
  {
    doDiff(structures, comparator, in);
  }


	return 0;
}

class TakeStructureAddress : public ::std::unary_function<PathStructurePair, ssc::Structure *>
{
public:
  ssc::Structure * operator()(PathStructurePair pair) const
  {
    return pair.second.get();
  }
};

void doUniques(const StructuresList & structures, ComparatorPtr comparator)
{
  typedef ::boost::transform_iterator<TakeStructureAddress, StructuresList::const_iterator> iterator;
  ssu::UniqueStructureSet structuresSet(comparator->getComparator());

  // First let's put all the structures in the set to see which are unique
  structuresSet.insert(iterator(structures.begin()), iterator(structures.end()));

  // Probably easiest just to iterate through the structures list as this has
  // the paths, and see which ones survived into the unique structure set

  // Annoying cast needed to stay consistent here
  const ssu::UniqueStructureSet * constSet = const_cast<const ssu::UniqueStructureSet *>(&structuresSet);
  ssu::UniqueStructureSet::const_iterator foundIt;
  const ssu::UniqueStructureSet::const_iterator end = constSet->end();
  BOOST_FOREACH(const PathStructurePair pair, structures)
  {
    foundIt = constSet->find(pair.second.get());
    if(foundIt != end)
    {
      std::cout << pair.first.string() << std::endl;
    }
  }

}


void doDiff(
  const StructuresList & structures,
  ComparatorPtr comparator,
  const InputOptions & in)
{
  const size_t numStructures = structures.size();
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
}
