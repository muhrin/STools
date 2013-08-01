/*
 * StructureBuild.cpp
 *
 *  Created on: Nov 10, 2011
 *      Author: Martin Uhrin
 */

// INCLUEDES /////////////
#include "build_cell/StructureBuild.h"

#include "build_cell/BuildAtomInfo.h"
#include "build_cell/GenSphere.h"
#include "build_cell/Sphere.h"
#include "build_cell/StructureContents.h"
#include "build_cell/SymmetryGroup.h"
#include "common/Constants.h"
#include "common/Structure.h"
#include "common/UnitCell.h"
#include "math/Random.h"

namespace sstbx {
namespace build_cell {

const double StructureBuild::RadiusCalculator::FALLBACK_RADIUS = 10.0;

StructureBuild::RadiusCalculator::RadiusCalculator(
  const double radius_,
  const bool isMultiplier_):
radius(radius_),
isMultiplier(isMultiplier_)
{}

double StructureBuild::RadiusCalculator::getRadius(const double vol) const
{
  if(isMultiplier)
  {
    if(vol != 0.0)
      return radius * Sphere::radius(vol);
    else
      return FALLBACK_RADIUS;
  }
  else
    return radius;
}

StructureBuild::StructureBuild(
  common::Structure & structure,
  const StructureContents & intendedContents,
  const RadiusCalculator & radiusCalculator):
myStructure(structure),
myIntendedContents(intendedContents)
{
  myGenShape.reset(new GenSphere(radiusCalculator.getRadius(myIntendedContents.getVolume())));
  myTransform.eye();
  myTransformCurrent = true;
  mySpeciesPairDistancesCurrent = true;
}

StructureBuild::StructureBuild(
  common::Structure & structure,
  const StructureContents & intendedContents,
  GenShapePtr genShape):
myStructure(structure),
myIntendedContents(intendedContents),
myGenShape(genShape)
{
  myTransform.eye();
  myTransformCurrent = true;
  mySpeciesPairDistancesCurrent = true;
}

common::Structure & StructureBuild::getStructure()
{
  return myStructure;
}

const common::Structure & StructureBuild::getStructure() const
{
  return myStructure;
}

size_t StructureBuild::getNumAtomInfos() const
{
  return myAtomsInfo.size();
}

StructureBuild::AtomInfoIterator StructureBuild::beginAtomInfo()
{
  return myAtomInfoList.begin();
}

StructureBuild::AtomInfoIterator StructureBuild::endAtomInfo()
{
  return myAtomInfoList.end();
}

BuildAtomInfo * StructureBuild::getAtomInfo(common::Atom & atom)
{
  const AtomInfoMap::iterator it = myAtomsInfo.find(&atom);

  if(it == myAtomsInfo.end())
    return NULL;

  return it->second;
}

const BuildAtomInfo * StructureBuild::getAtomInfo(common::Atom & atom) const
{
  const AtomInfoMap::const_iterator it = myAtomsInfo.find(&atom);

  if(it == myAtomsInfo.end())
    return NULL;

  return it->second;
}

BuildAtomInfo & StructureBuild::createAtomInfo(common::Atom & atom)
{
  BuildAtomInfo & atomInfo = *myAtomInfoList.insert(myAtomInfoList.end(), new BuildAtomInfo(atom));
  myAtomsInfo[&atom] = &atomInfo;
  return atomInfo;
}

void StructureBuild::addAtom(common::Atom & atom, BuildAtomInfo & atomInfo)
{
  myAtomsInfo[&atom] = &atomInfo;
}

void StructureBuild::removeAtom(common::Atom & atom)
{
  AtomInfoMap::iterator itMap = myAtomsInfo.find(&atom);
  SSLIB_ASSERT(itMap != myAtomsInfo.end());

  BuildAtomInfo & atomInfo = *itMap->second;
  AtomInfoList::iterator itList = myAtomInfoList.begin();
  for(AtomInfoList::iterator end = myAtomInfoList.end(); itList != end; ++itList)
  {
    if(&(*itList) == &atomInfo)
      break;
  }
  SSLIB_ASSERT(itList != myAtomInfoList.end());

  myAtomsInfo.erase(itMap);
  if(atomInfo.getNumAtoms() == 0)
    myAtomInfoList.erase(itList);
}

const IGeneratorShape & StructureBuild::getGenShape() const
{
  return *myGenShape;
}

const SymmetryGroup * StructureBuild::getSymmetryGroup() const
{
  return mySymmetryGroup.get();
}

void StructureBuild::setSymmetryGroup(SymmetryGroupPtr symGroup)
{
  mySymmetryGroup = symGroup;
}

StructureBuild::FixedSet StructureBuild::getFixedSet() const
{
  FixedSet fixedSet;

  BOOST_FOREACH(AtomInfoMap::const_reference atomInfo, myAtomsInfo)
  {
    if(atomInfo.second->isFixed())
      fixedSet.insert(atomInfo.first->getIndex());
  }

  return fixedSet;
}

bool StructureBuild::extrudeAtoms()
{
  return myAtomsExtruder.extrudeAtoms(myStructure, getFixedSet());
}

void StructureBuild::pushTransform(const ::arma::mat44 & transform)
{
  myTransformStack.push_back(transform);
  myTransformCurrent = false;
}

void StructureBuild::popTransform()
{
  SSLIB_ASSERT(!myTransformStack.empty());

  myTransformStack.pop_back();
  myTransformCurrent = false;
}

const ::arma::mat44 & StructureBuild::getTransform() const
{
  if(!myTransformCurrent)
  {
    // Do it this way to stop accumulation of error from lots of push/pops
    myTransform.eye();
    for(int i = 0; i < myTransformStack.size(); ++i)
      myTransform *= myTransformStack[i];
    myTransformCurrent = true;
  }
  return myTransform;
}

void StructureBuild::pushSpeciesPairDistances(const SpeciesPairDistances & distances)
{
  mySpeciesPairDistancesStack.push_back(distances);
  mySpeciesPairDistancesCurrent = false;
}

void StructureBuild::popSpeciesPairDistances()
{
  mySpeciesPairDistancesStack.pop_back();
  mySpeciesPairDistancesCurrent = false;
}
const StructureBuild::SpeciesPairDistances & StructureBuild::getSpeciesPairDistances() const
{
  typedef ::std::set< ::std::string> SpeciesSet;
  static const double UNINITIALISED = -1.0;

  if(!mySpeciesPairDistancesCurrent)
  {
    SpeciesSet allSpecies;
    BOOST_FOREACH(const SpeciesPairDistances & dist, mySpeciesPairDistancesStack)
    {
      BOOST_FOREACH(const ::std::string & species, dist.species)
      {
        allSpecies.insert(species);
      }
    }

    const size_t num = allSpecies.size();
    mySpeciesPairDistances.species.assign(allSpecies.begin(), allSpecies.end());
    mySpeciesPairDistances.distances.set_size(num, num);
    mySpeciesPairDistances.distances.fill(UNINITIALISED);
    ::std::map< ::std::string, int> indexMap;
    for(int i = 0; i < mySpeciesPairDistances.species.size(); ++i)
      indexMap[mySpeciesPairDistances.species[i]] = i;

    SpeciesSet::iterator it;
    const int totalPairs = num * (num - 1) / 2;
    int pairs = 0;
    int x, y;
    for(int i = mySpeciesPairDistancesStack.size() - 1; i >= 0 && pairs != totalPairs; --i)
    {
      const SpeciesPairDistances & current = mySpeciesPairDistancesStack[i];
      for(int row = 0; row < current.species.size(); ++row)
      {
        for(int col = row + 1; col < current.species.size(); ++col)
        {
          x = indexMap[current.species[row]];
          y = indexMap[current.species[col]];
          if(mySpeciesPairDistances.distances(x, y) != UNINITIALISED)
          {
            mySpeciesPairDistances.distances(x, y) = mySpeciesPairDistances.distances(y, x) = current.distances(row, col);
            ++pairs;
          }
        }
      }
    }

    mySpeciesPairDistancesCurrent = true;
  }
  return mySpeciesPairDistances;
}

}
}
