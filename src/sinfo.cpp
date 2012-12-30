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

// From StructurePipe
#include <io/StructureReadWriteManager.h>
#include <utility/PipeDataInitialisation.h>

// My includes //
#include "sinfo/Ancillary.h"
#include "sinfo/DataGatherer.h"
#include "utility/CustomTokens.h"

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

  // Set up the tokens that we know about
  TokensMap tokensMap;
  CustomisableTokens customisable = generateTokens(tokensMap);

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

  DataGatherer gatherer;
  ssc::AtomSpeciesDatabase speciesDb;
  ::std::string inputFile;
  ssio::ResourceLocator structureLocator;
  size_t numLoaded = 0;
  BOOST_FOREACH(inputFile, in.inputFiles)
  {
    if(structureLocator.set(inputFile))
    {
      const size_t numLoadedFromFile = rwMan.readStructures(
        structures,
        structureLocator,
        speciesDb);
      
      for(size_t i = numLoaded; i < numLoaded + numLoadedFromFile; ++i)
      {
        sortedKeys.push_back(&structures[i]);
        gatherer.gather(structures[i]);
      }
      numLoaded += numLoadedFromFile; // Up the counter
    }
  }

  // Set any values gathered from the collection of structures loaded
  ::boost::optional<double> energy;
  energy = gatherer.getLowestEnergy();
  if(energy)
    customisable.lowestEnergy->setRelativeEnergy(*energy);
  energy = gatherer.getLowestEnergyPerAtom();
  if(energy)
    customisable.lowestEnergyPerAtom->setRelativeEnergy(*energy);

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
    {
      it->second->sort(sortedKeys, infoTable);
    }
  }

  printInfo(infoTable, sortedKeys, tokensInfo, tokensMap, in);

  return 0;
}


