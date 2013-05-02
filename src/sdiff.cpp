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
#include <vector>

#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <boost/iterator/transform_iterator.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/program_options.hpp>
#include <boost/ptr_container/ptr_map.hpp>
#include <boost/tokenizer.hpp>

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
#include <math/RunningStats.h>
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
namespace ssm = ::sstbx::math;
namespace structure_properties = ssc::structure_properties;

// TYPEDEFS ////////////////////////////////////////
typedef ::boost::shared_ptr<ssc::Structure> SharedStructurePtr;
typedef ::ssio::StructuresContainer StructuresContainer;
typedef ::boost::shared_ptr<ssu::IBufferedComparator> ComparatorPtr;
typedef ::std::vector<size_t> InputGroup;

// CONSTANTS ////////////////////////////////////////

// CLASSES /////////////////////////////////////////
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
  double cutoffFactor;
};

class LocatorGroups
{
  static const ::std::string GROUP_DELIMITER;
public:
  typedef ::std::vector<size_t> LocatorGroup;
  
  void populate(const ::std::string & groupsString);
  const ssio::ResourceLocator & getLocator(const size_t locIdx) const;

  size_t numGroups() const;
  const LocatorGroup & getGroup(const size_t groupIdx) const;
private:
  int insertLocator(const ::std::string & locString);

  ::std::vector<ssio::ResourceLocator> myLocators;
  ::std::vector<LocatorGroup> myLocatorGroups;
};

template <typename GroupId>
class StructureGroups
{
public:
  typedef ::std::vector<ssc::Structure *> StructuresGroup;
  typedef ::std::map<GroupId, StructuresGroup> GroupsMap;
  typedef typename GroupsMap::const_reference const_reference;
  typedef typename GroupsMap::const_iterator const_iterator;

  void insertStructures(StructuresContainer & structures, const GroupId & groupId);
  const StructuresGroup * getGroup(const GroupId & groupId) const;

  size_t size() const;
  const_iterator begin() const;
  const_iterator end() const;

private:
  GroupsMap myGroups;
  StructuresContainer myStructures;
};

typedef StructureGroups<size_t> LoadedGroups;

const ::std::string LocatorGroups::GROUP_DELIMITER = ":";

// FORWARD DECLARES //////////
void preprocessStructure(ssc::Structure & structure, const ssio::ResourceLocator & loadLocation, const InputOptions & options);

void doPrintLists(const LoadedGroups & structures, ComparatorPtr comparator, const bool printUniques, const InputOptions & in);
void doPrintGroup(const LoadedGroups::StructuresGroup & structures, ComparatorPtr comparator, const bool printUniques, const InputOptions & in);

void doDiff(const LoadedGroups & structures, ComparatorPtr comparator, const InputOptions & in);
void doDiffGroups(
  const LoadedGroups::StructuresGroup & g1,
  const LoadedGroups::StructuresGroup & g2,
  ComparatorPtr comparator,
  const InputOptions & in
);

::sstbx::UniquePtr<ssu::IStructureComparator>::Type createComparator(const InputOptions & in);

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
      ("input-file", po::value< ::std::vector<::std::string> >(&in.inputFiles), "input file(s)")
      ("full", po::value<bool>(&in.printFull)->default_value(false)->zero_tokens(), "Print full matrix, not just lower triangular")
      ("maxatoms", po::value<unsigned int>(&in.maxAtoms)->default_value(12), "The maximum number of atoms before switching to fast comparison method.")
      ("comp,c", po::value< ::std::string>(&in.comparator)->default_value("sd"), "The comparator to use: sd = sorted distance, sdex = sorted distance extended, dm = distance matrix")
      ("agnostic,a", po::value<bool>(&in.volumeAgnostic)->default_value(false)->zero_tokens(), "Volume agnostic: volume/atom to 1 for each structure before performing comparison")
      ("no-primitive,p", po::value<bool>(&in.dontUsePrimitive)->default_value(false)->zero_tokens(), "Do not transform structures to primitive setting before comparison")
      ("mode,m", po::value<char>(&in.mode)->default_value('d'), "Mode:\nd = diff,\nu = print list of unique structures (first if duplicates),\ns = print list of similar structures (excluding first)")
      ("summary,s", po::value<bool>(&in.summaryOnly)->default_value(false)->zero_tokens(), "Show summary only")
      ("cutoff", po::value<double>(&in.cutoffFactor)->default_value(1.5), "Set the atom distances comparison cutoff as a multiple of the longest unit cell diagonal")
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
    ::std::cerr << "No structure files given.  Supply files with piped input or as command line paramter." << std::endl;
    return 1;
  }

  // Build up the input groups string from the input files array
  ::std::stringstream ss;
  BOOST_FOREACH(const ::std::string & inputFile, in.inputFiles)
    ss << inputFile << " ";
  LocatorGroups locGroups;
  locGroups.populate(ss.str());

  // Choose the comparator
  ::sstbx::UniquePtr<ssu::IStructureComparator>::Type comp = createComparator(in);
  if(!comp.get())
    return 1;
  
  ssc::AtomSpeciesDatabase speciesDb;
  ssio::StructureReadWriteManager rwMan;
  sp::utility::initStructureRwManDefault(rwMan);

  size_t totalLoaded = 0;
  StructureGroups<size_t> groups;
  for(size_t groupIdx = 0; groupIdx < locGroups.numGroups(); ++groupIdx)
  {
    const LocatorGroups::LocatorGroup & group = locGroups.getGroup(groupIdx);
    StructuresContainer loadedStructures;
    BOOST_FOREACH(const size_t locIdx, group)
    {
      const ssio::ResourceLocator & loc = locGroups.getLocator(locIdx);
      if(!fs::exists(loc.path()))
      {
        ::std::cerr << "File " << loc.path().string() << " does not exist.  Skipping" << ::std::endl;
        continue;
      }

      size_t lastLoaded = rwMan.readStructures(loadedStructures, loc, speciesDb);
      if(lastLoaded == 0)
        ::std::cerr << "Couldn't load structure(s) from " << loc.string() << ::std::endl;
      else
      {
        // Perform any preprocessing on the loaded structure
        for(size_t i = 0; i < lastLoaded; ++i)
          preprocessStructure(loadedStructures[i], loc, in);

        totalLoaded += lastLoaded;
        groups.insertStructures(loadedStructures, groupIdx);
      }
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
    doPrintLists(groups, comparator, true, in);
  else if(in.mode == 's')
    doPrintLists(groups, comparator, false, in);
  else if(in.mode == 'd')
    doDiff(groups, comparator, in);
  else
  {
    std::cerr << "Error: Unrecognised mode - " << in.mode << std::endl;
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

void doPrintLists(
  const LoadedGroups & groups,
  ComparatorPtr comparator,
  const bool printUniques,
  const InputOptions & in
)
{
  BOOST_FOREACH(LoadedGroups::const_reference group, groups)
    doPrintGroup(group.second, comparator, printUniques, in);
  if(groups.size() > 1)
    ::std::cout << ::std::endl;
}

void doPrintGroup(
  const LoadedGroups::StructuresGroup & group,
  ComparatorPtr comparator,
  const bool printUniques,
  const InputOptions & in
)
{
  typedef ::std::pair<ssu::UniqueStructureSet<>::iterator, bool> InsertReturnVal;
  ssu::UniqueStructureSet<> structuresSet(comparator->getComparator());

  InsertReturnVal insertResult;

  BOOST_FOREACH(ssc::Structure * structure, group)
  {
    insertResult = structuresSet.insert(structure);
    if(!in.summaryOnly && insertResult.second == printUniques)
    {
      const ssio::ResourceLocator * locator = structure->getProperty(structure_properties::io::LAST_ABS_FILE_PATH);
      if(locator)
        std::cout << ssio::relative(*locator).string() << std::endl;
      else
        ::std::cerr << "Error: couldn't find save path for structure " << structure->getName() << ::std::endl;
    }
  }

  if(in.summaryOnly)
  {
    ::std::cout << "total: " << group.size() << ", unique: " << structuresSet.size()
      << ", similar: " << group.size() - structuresSet.size() << ::std::endl;
  }
}

void doDiff(
  const LoadedGroups & groups,
  ComparatorPtr comparator,
  const InputOptions & in)
{
  const LoadedGroups::const_iterator end = groups.end();
  for(LoadedGroups::const_iterator g1It = groups.begin(); g1It != end; ++g1It)
  {
    // Don't diff a group with itself unless there is only one group
    LoadedGroups::const_iterator g2It = g1It;
    if(groups.size() > 1)
      ++g2It;
    for(; g2It != end; ++g2It)
    {
      doDiffGroups(g1It->second, g2It->second, comparator, in);
    }
  }
  if(groups.size() > 1)
    ::std::cout << ::std::endl;
}

void doDiffGroups(
  const LoadedGroups::StructuresGroup & g1,
  const LoadedGroups::StructuresGroup & g2,
  ComparatorPtr comparator,
  const InputOptions & in
)
{
  typedef ::std::vector<ssu::IBufferedComparator::ComparisonDataHandle> ComparisonHandles;

  ComparisonHandles g1ComparisonHandles(g1.size());
  ComparisonHandles g2ComparisonHandles(g2.size());

  ::arma::mat diffs(g1.size(), g1.size());
  diffs.diag().fill(0.0);

  size_t i;
  for(i = 0; i < g1.size(); ++i)
  {
    g1ComparisonHandles[i] = comparator->generateComparisonData(*g1[i]);
  }
  for(i = 0; i < g2.size(); ++i)
  {
    g2ComparisonHandles[i] = comparator->generateComparisonData(*g2[i]);
  }

  double mean = 0.0, min = ::std::numeric_limits<double>::max(), max = 0.0;
  ssm::RunningStats stats;
  for(size_t i = 0; i < g1.size(); ++i)
  {
    for(size_t j = 0; j < g2.size(); ++j)
    {
      diffs(i, j) = comparator->compareStructures(
        g1ComparisonHandles[i],
        g2ComparisonHandles[j]
      );
      
      stats.insert(diffs(i, j));
    }
  }
  diffs = ::arma::symmatu(diffs);

  if(in.summaryOnly)
  {  
    ::std::cout << "total: " << stats.num() <<
      ", mean: " << stats.mean() <<
      ", min: "<< stats.min() <<
      ", max: " << stats.max() << ::std::endl;
  }
  else
  {
    for(size_t i = 0; i < g1.size(); ++i)
    {
      for(size_t j = 0; j < g2.size(); ++j)
      {
        ::std::cout << diffs(i, j) << "\t";
      }
      ::std::cout << ::std::endl;
    }
  }
}

::sstbx::UniquePtr<ssu::IStructureComparator>::Type createComparator(const InputOptions & in)
{
  ::sstbx::UniquePtr<ssu::IStructureComparator>::Type comp;
  if(in.comparator == "sd")
  {
    ::sstbx::UniquePtr<ssu::SortedDistanceComparator>::Type
      sortedDist(new ssu::SortedDistanceComparator(in.tolerance, false, false));
    sortedDist->setCutoffFactor(in.cutoffFactor);
    comp.reset(sortedDist.release());
  }
  else if(in.comparator == "sdex")
    comp.reset(new ssu::SortedDistanceComparatorEx());
  else if(in.comparator == "dm")
    comp.reset(new ssu::DistanceMatrixComparator(in.tolerance, in.maxAtoms));
  else
    ::std::cerr << "Error: unrecognised comparator - " << in.comparator << ::std::endl;
  return comp;
}

void LocatorGroups::populate(const ::std::string & groupString)
{
  typedef boost::tokenizer<boost::char_separator<char> > Tok;
  static const boost::char_separator<char> GROUP_SEP(GROUP_DELIMITER.c_str());
  static const boost::char_separator<char> GROUP_ENTRY_SEP(" \t\n");

  Tok groupsTok(groupString, GROUP_SEP);
  int index;
  BOOST_FOREACH(const ::std::string & inputGroupString, groupsTok)
  {
    Tok entryTok(inputGroupString, GROUP_ENTRY_SEP);
    LocatorGroup group;
    BOOST_FOREACH(::std::string groupEntryString, entryTok)
    {
      ::boost::trim(groupEntryString);
      index = insertLocator(groupEntryString);
      if(index >= 0)
        group.push_back(static_cast<size_t>(index));
    }
    if(!group.empty())
      myLocatorGroups.push_back(group);
  }
}

const ssio::ResourceLocator & LocatorGroups::getLocator(const size_t locIdx) const
{
  return myLocators[locIdx];
}

size_t LocatorGroups::numGroups() const
{
  return myLocatorGroups.size();
}

const LocatorGroups::LocatorGroup & LocatorGroups::getGroup(const size_t groupIdx) const
{
  return myLocatorGroups[groupIdx];
}

int LocatorGroups::insertLocator(const ::std::string & locString)
{
  ssio::ResourceLocator loc;
  if(!loc.set(locString)) // Check if it's a valid locator string
    return -1;

  // First check if we already have an equivalent locator
  int i;
  for(i = 0; i < static_cast<int>(myLocators.size()); ++i)
  {
    if(ssio::equivalent(loc, myLocators[i]))
      break;
  }
  if(i == myLocators.size()) // Couldn't find an equivalent locator
    myLocators.push_back(loc);

  return i;
}

template <typename GroupId>
void StructureGroups<GroupId>::insertStructures(
  StructuresContainer & structures,
  const GroupId & groupId
)
{
  BOOST_FOREACH(ssc::Structure & structure, structures)
  {
    myGroups[groupId].push_back(&structure);
  }
  // Take ownership
  myStructures.transfer(myStructures.end(), structures);
}

template <typename GroupId>
const typename StructureGroups<GroupId>::StructuresGroup *
StructureGroups<GroupId>::getGroup(const GroupId & groupId) const
{
  const LocatorGroup::const_iterator it = myGroups.find(groupId);
  if(it == myGroups.end())
    return NULL;
  
  return &(*it);
}

template <typename GroupId>
size_t StructureGroups<GroupId>::size() const
{
  return myGroups.size();
}

template <typename GroupId>
typename StructureGroups<GroupId>::const_iterator
StructureGroups<GroupId>::begin() const
{
  return myGroups.begin();
}

template <typename GroupId>
typename StructureGroups<GroupId>::const_iterator
StructureGroups<GroupId>::end() const
{
  return myGroups.end();
}
