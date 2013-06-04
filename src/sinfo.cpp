/*
 * sinfo.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////

// From SSLib //
#include <common/AtomSpeciesDatabase.h>
#include <common/Structure.h>
#include <common/StructureProperties.h>
#include <common/Types.h>
#include <common/UnitCell.h>
#include <io/ResourceLocator.h>
#include <io/StructureReadWriteManager.h>
#include <utility/SortedDistanceComparator.h>
#include <utility/UniqueStructureSet.h>

// From StructurePipe
#include <utility/PipeDataInitialisation.h>

// stools_common 
#include <utility/CustomTokens.h>

// My includes //
#include "sinfo/Ancillary.h"

// NAMESPACES ////////////////////////////////
using namespace ::stools::sinfo;
namespace sp  = ::spipe;
namespace ssu = ::sstbx::utility;
namespace ssc = ::sstbx::common;
namespace ssio = ::sstbx::io;

int main(const int argc, char * argv[])
{
  typedef ssio::StructuresContainer StructuresContainer;

  ssio::StructureReadWriteManager rwMan;
  sp::utility::initStructureRwManDefault(rwMan);

  ssc::AtomSpeciesDatabase speciesDb;

  // Set up the tokens that we know about
  TokensMap tokensMap;
  CustomisableTokens customisable = generateTokens(tokensMap, speciesDb);

  // Process input and detect errors
  InputOptions in;
  Result::Value result = processInputOptions(in, argc, argv, tokensMap);
  if(result != Result::SUCCESS)
    return result;

  // Now get the tokens requested by the user
  ::std::string formatString;
  TokensInfo tokensInfo;
  result = getRequiredTokens(tokensInfo, tokensMap, in);
  if(result != Result::SUCCESS)
    return result;

  StructureInfoTable infoTable;
  StructuresContainer structures;

  SortedKeys sortedKeys;

  ::std::string inputFile;
  ssio::ResourceLocator structureLocator;
  size_t numKept, numLoaded = 0;
  BOOST_FOREACH(inputFile, in.inputFiles)
  {
    if(structureLocator.set(inputFile))
    {
      const size_t numLoadedFromFile = rwMan.readStructures(
        structures,
        structureLocator,
        speciesDb
      );
      
      numKept = numLoadedFromFile;

      // Pre-process the structure
      for(size_t i = numLoaded; i < numLoaded + numKept; ++i)
        sortedKeys.push_back(&structures[i]);

      numLoaded += numKept; // Up the counter
    }
  }

  if(structures.empty())
    return 0;

  // Populate the information table
  BOOST_FOREACH(const ssc::Structure & structure, structures)
  {
    BOOST_FOREACH(const ::std::string & tokenEntry, tokensInfo.tokenStrings)
    {
      tokensMap.at(tokenEntry).insert(infoTable, structure);
    }
    if(!in.sortToken.empty())
      tokensMap.at(in.sortToken).insert(infoTable, structure);
  }

  // Sort the structures if requested
  if(!in.sortToken.empty())
  {
    const TokensMap::const_iterator it = tokensMap.find(in.sortToken);
    if(it != tokensMap.end())
      it->second->sort(sortedKeys, infoTable, in.reverseSortComparison);
  }

  // Set relative values
  // TODO: Check which of these are used from the tokens map and only apply those
  customisable.lowestEnergy->setRelativeTo(*sortedKeys.front());
  customisable.lowestEnergyPerAtom->setRelativeTo(*sortedKeys.front());
  customisable.lowestEnthalpy->setRelativeTo(*sortedKeys.front());
  customisable.lowestEnthalpyPerAtom->setRelativeTo(*sortedKeys.front());
  // Update the table with the new relative values
  BOOST_FOREACH(const ssc::Structure & structure, structures)
  {
    customisable.lowestEnergy->insert(infoTable, structure);
    customisable.lowestEnergyPerAtom->insert(infoTable, structure);
    customisable.lowestEnthalpy->insert(infoTable, structure);
    customisable.lowestEnthalpyPerAtom->insert(infoTable, structure);
  }

  if(in.uniqueMode)
  {
    ssu::UniqueStructureSet<ssc::Structure *> uniqueStructures(
      ssu::IStructureComparatorPtr(new ssu::SortedDistanceComparator(in.uniqueTolerance))
    );
    int idx = 0;
    SortedKeys::iterator it = sortedKeys.begin();
    while(it != sortedKeys.end())
    {
      // Have we seen this structure before?
      // TODO: Have to remove this const cast by fixing unique structures to accept const Structure as well
      if(uniqueStructures.insert(const_cast<ssc::Structure * >(*it)).second == false)
      {
        infoTable.eraseRow(*it);
        it = sortedKeys.erase(it);
      }
      else
        ++it;
    }
  }

  const size_t numToPrint = in.printTop == PRINT_ALL ? sortedKeys.size() : ::std::min(sortedKeys.size(), (size_t)in.printTop);
  printInfo(infoTable, sortedKeys, tokensInfo, tokensMap, in, numToPrint);

  return 0;
}


