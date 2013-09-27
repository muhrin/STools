/*
 * sinfo.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include <spl/SSLib.h>
#ifdef SSLIB_USE_CGAL
#  include <spl/analysis/ConvexHullStructures.h>
#endif
#include <spl/common/Types.h>
#include <spl/common/UnitCell.h>
#include <spl/io/ResourceLocator.h>
#include <spl/io/StructureReadWriteManager.h>
#include <spl/utility/SortedDistanceComparator.h>
#include <spl/utility/UniqueStructureSet.h>

// From StructurePipe
#include <utility/PipeDataInitialisation.h>

// stools_common 
#include <utility/CustomTokens.h>

// My includes //
#include "sinfo/Ancillary.h"

// NAMESPACES ////////////////////////////////
using namespace ::stools::sinfo;
namespace sp = ::spipe;
#ifdef SSLIB_USE_CGAL
namespace ssa = ::spl::analysis;
#endif
namespace ssu = ::spl::utility;
namespace ssc = ::spl::common;
namespace ssio = ::spl::io;

void
addToken(const ::std::string & token, InputOptions & in);

int
main(const int argc, char * argv[])
{
  typedef ssio::StructuresContainer StructuresContainer;

  ssio::StructureReadWriteManager rwMan;
  sp::utility::initStructureRwManDefault(rwMan);

  // Set up all the tokens that we know about
  TokensMap tokensMap;
  CustomisableTokens customisable = generateTokens(tokensMap);

  // Process input and detect errors
  InputOptions in;
  Result::Value result = processInputOptions(in, argc, argv, tokensMap);
  if(result != Result::SUCCESS)
    return result;

  // Now get the tokens requested by the user
  const bool autoTokens = in.infoString == INFO_STRING_AUTO;
  if(autoTokens)
    in.infoString = DEFAULT_INFO_STRING;
  TokensInfo tokensInfo;

  // Reprocess the tokens just in case any have been added
  BOOST_FOREACH(const ::std::string & additionalToken, in.additionalTokens)
  {
    addToken(additionalToken, in);
  }
  result = getRequiredTokens(tokensInfo, tokensMap, in);
  if(result != Result::SUCCESS)
    return result;

  StructureInfoTable infoTable;
  StructuresContainer structures;
  SortedKeys sortedKeys;

  ssc::AtomsFormula filterFormula;
  if(!in.filterString.empty())
  {
    if(!filterFormula.fromString(in.filterString))
      ::std::cerr << "Failed to parse filter string: " << in.filterString
          << ::std::endl;
  }
  FormulaFilter formulaFilter(filterFormula);

  ::std::string inputFile;
  ssio::ResourceLocator structureLocator;
  BOOST_FOREACH(inputFile, in.inputFiles)
  {
    if(structureLocator.set(inputFile))
    {
      StructuresContainer loaded;
      rwMan.readStructures(loaded, structureLocator);

      structures.transfer(structures.end(), loaded);
    }
  }

  if(structures.empty())
    return 1; // No structures found

#ifdef SSLIB_USE_CGAL
  const bool useHullDist =
      in.maxHullDist != MAX_HULL_DIST_IGNORE || tokensInfo.getToken("hd") != NULL;
  if(in.stableCompositions || useHullDist)
  {
    // Generate the convex hull
    ssa::ConvexHullStructures hullStructures(
        ssa::ConvexHullStructures::generateEndpoints(structures.begin(),
            structures.end()));
    for(StructuresContainer::iterator it = structures.begin();
        it != structures.end(); /* increment in loop body */)
    {
      if(hullStructures.insertStructure(*it) == hullStructures.structuresEnd())
        it = structures.erase(it); // Erase structures that aren't in the space of the hull
      else
        ++it;
    }
    // Add the formation enthalpies to the structures properties
    hullStructures.populateFormationEnthalpies();
    if(autoTokens)
      addToken("hf", in);

    if(useHullDist)
      hullStructures.populateHullDistances();

    if(in.stableCompositions)
    {
      for(StructuresContainer::iterator it = structures.begin();
          it != structures.end();
          /* increment in loop body */)
      {
        if(hullStructures.getStability(*it)
            != ssa::ConvexHullStructures::Stability::STABLE)
          it = structures.erase(it);
        else
          ++it;
      }
    }
    else if(in.maxHullDist != MAX_HULL_DIST_IGNORE)
    {
      // Now go through getting those that are within the maximum formation enthalpy
      for(StructuresContainer::iterator it = structures.begin();
          it != structures.end(); /* increment in loop body */)
      {
        const double * const hullDist =
            it->getProperty(ssc::structure_properties::general::HULL_DISTANCE);
        if(!hullDist || *hullDist > in.maxHullDist)
          it = structures.erase(it);
        else
          ++it;
      }
    }
  }
#endif

  // Filter out any that we don't want
  structures.erase(
      ::std::remove_if(structures.begin(), structures.end(), formulaFilter),
      structures.end());

  if(in.compositionTop != 0)
  {
    typedef ssio::StructuresContainer::iterator StructuresIterator;
    typedef ::std::map< double, StructuresIterator> TopN;
    typedef ::std::map< ssc::AtomsFormula, TopN> FormulasMap;

    FormulasMap formulasMap;
    ::std::set< StructuresIterator> toRemove;
    ssc::AtomsFormula formula;

    double enthalpyPerAtom;
    for(StructuresIterator it = structures.begin(), end = structures.end();
        it != end; ++it)
    {
      const double * const enthalpy = it->getProperty(
          ssc::structure_properties::general::ENTHALPY);
      if(!enthalpy)
      {
        toRemove.insert(it);
        continue;
      }
      enthalpyPerAtom = *enthalpy / static_cast< double>(it->getNumAtoms());

      formula = it->getComposition();
      formula.reduce();

      if(!formulasMap[formula].insert(::std::make_pair(enthalpyPerAtom, it)).second)
        toRemove.insert(it);
    }

    BOOST_FOREACH(FormulasMap::reference form, formulasMap)
    {
      if(static_cast<int>(form.second.size()) > in.compositionTop)
      {
        TopN::reverse_iterator it = form.second.rbegin();
        for(int i = 0; i < static_cast<int>(form.second.size()) - in.compositionTop; ++i, ++it)
          toRemove.insert(it->second);
      }
    }

    for(::std::set< StructuresIterator>::reverse_iterator it =
        toRemove.rbegin(), end = toRemove.rend(); it != end; ++it)
    {
      structures.erase(*it);
    }
  }

  if(structures.empty())
    return 0;

  // Reprocess the tokens just in case any have been added
  tokensInfo.formatStrings.clear();
  tokensInfo.tokens.clear();
  result = getRequiredTokens(tokensInfo, tokensMap, in);
  if(result != Result::SUCCESS)
    return result;

  // Preprocess structures
  BOOST_FOREACH(ssc::Structure & structure, structures)
  {
    sortedKeys.push_back(&structure);
  }

  // Populate the information table
  BOOST_FOREACH(const ssc::Structure & structure, structures)
  {
    if(!filterFormula.isEmpty()
        && structure.getComposition().numMultiples(filterFormula) == -1)
      continue;
    BOOST_FOREACH(const stools::utility::InfoToken * token, tokensInfo.tokens)
    {
      token->insert(infoTable, structure);
    }

    if(tokensInfo.sortToken)
      tokensInfo.sortToken->insert(infoTable, structure);
  }

  // Sort the structures if requested
  if(tokensInfo.sortToken)
    tokensInfo.sortToken->sort(sortedKeys, infoTable, in.reverseSortComparison);

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
    ssu::SortedDistanceComparator::ConstructionInfo constructInfo;
    constructInfo.tolerance = in.uniqueTolerance;
    ssu::UniqueStructureSet< ssc::Structure *> uniqueStructures(
        ssu::IStructureComparatorPtr(
            new ssu::SortedDistanceComparator(constructInfo)));

    SortedKeys::iterator it = sortedKeys.begin();
    while(it != sortedKeys.end())
    {
      // Have we seen this structure before?
      // TODO: Have to remove this const cast by fixing unique structures to accept const Structure as well
      if(uniqueStructures.insert(const_cast< ssc::Structure *>(*it)).second
          == false)
      {
        infoTable.eraseRow(*it);
        it = sortedKeys.erase(it);
      }
      else
        ++it;
    }
  }

  const size_t numToPrint =
      in.printTop == PRINT_ALL ?
          sortedKeys.size() :
          ::std::min(sortedKeys.size(), (size_t) in.printTop);
  printInfo(infoTable, sortedKeys, tokensInfo, in, numToPrint);

  return 0;
}

void
addToken(const ::std::string & token, InputOptions & in)
{
  if(!in.infoString.empty())
    in.infoString += "";
  in.infoString += "$" + token + "$";
}

