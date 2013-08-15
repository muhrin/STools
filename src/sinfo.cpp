/*
 * sinfo.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include <algorithm>

// From SSLib //
#include <analysis/ConvexHullStructures.h>
#include <common/AtomsFormula.h>
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
namespace sp = ::spipe;
namespace ssa = ::sstbx::analysis;
namespace ssu = ::sstbx::utility;
namespace ssc = ::sstbx::common;
namespace ssio = ::sstbx::io;

class FormulaFilter : public ::std::unary_function< const ssc::Structure &, bool>
{
public:
  FormulaFilter(const ssc::AtomsFormula & formula) :
      myFormula(formula)
  {
  }

  bool
  operator ()(const ssc::Structure & structure)
  {
    return !myFormula.isEmpty()
        && structure.getComposition().numMultiples(myFormula) == 0;
  }
private:
  const ssc::AtomsFormula myFormula;
};

int
main(const int argc, char * argv[])
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

  if(in.stableCompositions || in.maxHullDist != MAX_HULL_DIST_IGNORE)
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

    if(in.stableCompositions)
    {
      for(StructuresContainer::iterator it = structures.begin(); it != structures.end();
          /* increment in loop body */)
      {
        if(hullStructures.getStability(*it) != ssa::ConvexHullStructures::Stability::STABLE)
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
        ::sstbx::OptionalDouble dist = hullStructures.distanceToHull(*it);
        if(!dist || *dist > in.maxHullDist)
          it = structures.erase(it);
        else
          ++it;
      }
    }
  }


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
      if(form.second.size() > in.compositionTop)
      {
        TopN::reverse_iterator it = form.second.rbegin();
        for(int i = 0; i < form.second.size() - in.compositionTop; ++i, ++it)
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
    ssu::UniqueStructureSet< ssc::Structure *> uniqueStructures(
        ssu::IStructureComparatorPtr(
            new ssu::SortedDistanceComparator(in.uniqueTolerance)));
    int idx = 0;
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
  printInfo(infoTable, sortedKeys, tokensInfo, tokensMap, in, numToPrint);

  return 0;
}

