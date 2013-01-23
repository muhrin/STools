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
#include "common/Constants.h"
#include "common/Structure.h"
#include "common/UnitCell.h"
#include "common/Utils.h"

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

BuildAtomInfo * StructureBuild::getAtomInfo(common::Atom & atom)
{
  const AtomInfoMap::iterator it = myAtomsInfo.find(&atom);

  if(it == myAtomsInfo.end())
    return NULL;

  return &it->second;
}

const BuildAtomInfo * StructureBuild::getAtomInfo(common::Atom & atom) const
{
  const AtomInfoMap::const_iterator it = myAtomsInfo.find(&atom);

  if(it == myAtomsInfo.end())
    return NULL;

  return &it->second;
}

void StructureBuild::insertAtomInfo(BuildAtomInfo & atomInfo)
{
  myAtomsInfo.insert(::std::make_pair(&atomInfo.getAtom(), atomInfo));
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
    ::arma::vec3 point;
    point.randn();
    return common::randDouble(myClusterRadius) * point;
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

StructureBuild::FixedSet StructureBuild::getFixedSet() const
{
  FixedSet fixedSet;

  BOOST_FOREACH(AtomInfoMap::const_reference atomInfo, myAtomsInfo)
  {
    if(atomInfo.second.isFixed())
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
