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
#include <limits>
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
#include <io/BoostFilesystem.h>
#include <io/ResourceLocator.h>
#include <io/StructureReadWriteManager.h>
#include <utility/DistanceMatrixComparator.h>
#include <utility/SortedDistanceComparator.h>
#include <utility/SortedDistanceComparatorEx.h>
#include <utility/IBufferedComparator.h>
#include <utility/TransformFunctions.h>
#include <utility/UniqueStructureSet.h>

// From StructurePipe
#include <utility/PipeDataInitialisation.h>

// Local includes //
#include "utility/TerminalFunctions.h"


// NAMESPACES ////////////////////////////////
namespace fs = ::boost::filesystem;
namespace po = ::boost::program_options;
namespace sp = ::spipe;
namespace ssu = ::sstbx::utility;
namespace ssc = ::sstbx::common;
namespace ssio = ::sstbx::io;
namespace structure_properties = ssc::structure_properties;

typedef ::boost::shared_ptr<ssc::Structure> SharedStructurePtr;
typedef ::ssio::StructuresContainer StructuresContainer;
typedef ::boost::shared_ptr<ssu::IBufferedComparator> ComparatorPtr;

struct InputOptions
{
  double tolerance;
  ::std::vector< ::std::string> inputFiles;
  ::std::string comparator;
  bool printFull;
  unsigned int maxAtoms;
  char mode;
  bool volumeAgnostic;
  bool dontUsePrimitive;
  bool summaryOnly;
};

// FORWARD DECLARES //////////
void preprocessStructure(ssc::Structure & structure, const ssio::ResourceLocator & loadLocation, const InputOptions & options);
void doPrintList(StructuresContainer & structures, ComparatorPtr comparator, const bool printUniques, const InputOptions & in);
void doDiff(const StructuresContainer & structures, ComparatorPtr comparator, const InputOptions & in);

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
      ("tol,t", po::value<double>(&in.tolerance)->default_value(ssu::SortedDistanceComparator::DEFAULT_TOLERANCE), "Set comparator tolerance")
      ("input-file", po::value< ::std::vector< ::std::string> >(&in.inputFiles), "input file(s)")
      ("full", po::value<bool>(&in.printFull)->default_value(false)->zero_tokens(), "Print full matrix, not just lower triangular")
      ("maxatoms", po::value<unsigned int>(&in.maxAtoms)->default_value(12), "The maximum number of atoms before switching to fast comparison method.")
      ("comp,c", po::value< ::std::string>(&in.comparator)->default_value("sd"), "The comparator to use: sd = sorted distance, sdex = sorted distance extended, dm = distance matrix")
      ("agnostic,a", po::value<bool>(&in.volumeAgnostic)->default_value(false)->zero_tokens(), "Volume agnostic: volume/atom to 1 for each structure before performing comparison")
      ("no-primitive,p", po::value<bool>(&in.dontUsePrimitive)->default_value(false)->zero_tokens(), "Do not transform structures to primitive setting before comparison")
      ("mode,m", po::value<char>(&in.mode)->default_value('d'), "Mode:\nd = diff,\nu = print list of unique structures (first if duplicates),\ns = print list of similar structures (excluding first)")
      ("summary,s", po::value<bool>(&in.summaryOnly)->default_value(false)->zero_tokens(), "Show summary only")
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
    comp.reset(new ssu::SortedDistanceComparator(in.tolerance, false, false));
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
  ssio::StructureReadWriteManager rwMan;
  sp::utility::initStructureRwManDefault(rwMan);
  StructuresContainer loadedStructures;

  size_t lastLoaded, totalLoaded = 0;
  ssio::ResourceLocator locator;
  BOOST_FOREACH(const ::std::string & locatorString, in.inputFiles)
  {
    if(!locator.set(locatorString))
    {
      ::std::cerr << "Locator " << locatorString << " is not valid.  Skipping" << ::std::endl;
      continue;
    }
    if(!fs::exists(locator.path()))
    {
      ::std::cerr << "File " << locator.path().string() << " does not exist.  Skipping" << ::std::endl;
      continue;
    }

    lastLoaded = rwMan.readStructures(loadedStructures, locator, speciesDb);

    // TODO: If the structure has no atoms, remove it

    if(lastLoaded == 0)
      ::std::cerr << "Couldn't load structure(s) from " << locator.string() << ::std::endl;
    else
    {
      // Perform any preprocessing on the loaded structure
      for(size_t i = 0; i < lastLoaded; ++i)
        preprocessStructure(loadedStructures[i], locator, in);

      totalLoaded += lastLoaded;
    }
  }

  if(totalLoaded < 2)
  {
    ::std::cerr << "Not enough structures to compare." << ::std::endl;
    return 1;
  }

  ComparatorPtr comparator = comp->generateBuffered();


  // Do the actual comparison based on command line options
  if(in.mode == 'u')
    doPrintList(loadedStructures, comparator, true, in);
  else if(in.mode == 's')
    doPrintList(loadedStructures, comparator, false, in);
  else if(in.mode == 'd')
    doDiff(loadedStructures, comparator, in);
  else
  {
    std::cout << "Error: Unrecognised mode - " << in.mode << std::endl;
    return 1;
  }


	return 0;
}

void preprocessStructure(
  ssc::Structure & structure,
  const ssio::ResourceLocator & loadLocation,
  const InputOptions & options)
{
  // Make sure we know where we loaded the file from
  if(!structure.getProperty(structure_properties::io::LAST_ABS_FILE_PATH))
    structure.setProperty(structure_properties::io::LAST_ABS_FILE_PATH, ssio::absolute(loadLocation));

  if(!options.dontUsePrimitive)
    structure.makePrimitive();

  // If we are to be volume agnostic then set the volume
  // to 1.0 per atom
  ssc::UnitCell * const unitCell = structure.getUnitCell();
  if(options.volumeAgnostic && unitCell)
  {
    const double scaleFactor = structure.getNumAtoms() / unitCell->getVolume();
    structure.scale(scaleFactor);
  }
}

void doPrintList(
  StructuresContainer & structures,
  ComparatorPtr comparator,
  const bool printUniques,
  const InputOptions & in)
{
  typedef ::std::pair<ssu::UniqueStructureSet<>::iterator, bool> InsertReturnVal;
  ssu::UniqueStructureSet<> structuresSet(comparator->getComparator());

  InsertReturnVal insertResult;

  BOOST_FOREACH(ssc::Structure & structure, structures)
  {
    insertResult = structuresSet.insert(&structure);
    if(!in.summaryOnly && insertResult.second == printUniques)
    {
      const ssio::ResourceLocator * locator = structure.getProperty(structure_properties::io::LAST_ABS_FILE_PATH);
      if(locator)
        std::cout << ssio::relative(*locator).string() << std::endl;
      else
        ::std::cerr << "Error: couldn't find save path for structure " << structure.getName() << ::std::endl;
    }
  }

  if(in.summaryOnly)
  {
    ::std::cout << "total: " << structures.size() << ", unique: " << structuresSet.size()
      << ", similar: " << structures.size() - structuresSet.size() << ::std::endl;
  }
}

void doDiff(
  const StructuresContainer & structures,
  ComparatorPtr comparator,
  const InputOptions & in)
{
  typedef ::std::vector<ssu::IBufferedComparator::ComparisonDataHandle> ComparisonHandles;

  const size_t numStructures = structures.size();
  ComparisonHandles comparisonHandles(numStructures);
  ::arma::mat diffs(numStructures, numStructures);
  diffs.diag().fill(0.0);

  double mean = 0.0, min = ::std::numeric_limits<double>::max(), max = 0.0;
  for(size_t i = 0; i < numStructures; ++i)
  {
    comparisonHandles[i] = comparator->generateComparisonData(structures[i]);
  }
  for(size_t i = 0; i < numStructures - 1; ++i)
  {
    for(size_t j = i + 1; j < numStructures; ++j)
    {
      diffs(i, j) = comparator->compareStructures(comparisonHandles[i], comparisonHandles[j]);
      
      mean += diffs(i, j);
      max = ::std::max(max, diffs(i, j));
      min = ::std::min(min, diffs(i, j));
    }
  }
  diffs = ::arma::symmatu(diffs);
  mean /= 0.5 * numStructures * (numStructures - 1);

  if(in.summaryOnly)
  {  
    ::std::cout << "total: " << numStructures << ", mean: " << mean << ", min: " << min << ", max: " << max << ::std::endl;
  }
  else
  {
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
}
