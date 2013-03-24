/*
 * StructureBuild.cpp
 *
 *  Created on: Nov 10, 2011
 *      Author: Martin Uhrin
 */

// INCLUEDES /////////////
#include "build_cell/StructureBuild.h"

#include "build_cell/BuildAtomInfo.h"
#include "build_cell/Sphere.h"
#include "build_cell/StructureContents.h"
#include "build_cell/SymmetryGroup.h"
#include "common/Constants.h"
#include "common/Structure.h"
#include "common/UnitCell.h"
#include "common/Utils.h"
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
  myClusterRadius = radiusCalculator.getRadius(myIntendedContents.getVolume());
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

::arma::vec3 StructureBuild::getRandomPoint() const
{
  const common::UnitCell * cell = myStructure.getUnitCell();
  if(cell)
  {
    return cell->randomPoint();
  }
  else
  {
    // TODO: Replace this with gen sphere
    ::arma::vec3 point;
    point.randu();
    point *= 1.0 / sqrt(::arma::dot(point, point)); // normalise
    return math::randu(myClusterRadius) * point;
  }

}

double StructureBuild::getClusterRadius() const
{
  return myClusterRadius;
}

void StructureBuild::setClusterRadius(const RadiusCalculator & radiusCalculator)
{
  myClusterRadius = radiusCalculator.getRadius(myIntendedContents.getVolume());
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

}
}
