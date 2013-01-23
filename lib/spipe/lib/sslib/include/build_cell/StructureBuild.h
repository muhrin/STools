/*
 * StructureBuild.h
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef STRUCTURE_BUILD_H
#define STRUCTURE_BUILD_H

// INCLUDES ////////////
#include "SSLib.h"

#include <map>
#include <set>

#include "build_cell/BuildAtomInfo.h"
#include "build_cell/AtomExtruder.h"

namespace sstbx {
namespace common {
class Atom;
class Structure;
}
namespace build_cell {

class BuildAtomInfo;
class StructureContents;

class StructureBuild
{
public:

  typedef ::std::set<size_t> FixedSet;

  class RadiusCalculator
  {
  public:
    static const double FALLBACK_RADIUS;

    RadiusCalculator(const double radius_ = 2.5, const bool isMultiplier_ = true);
    double getRadius(const double volume) const;

    double radius;
    bool isMultiplier;
  };

  StructureBuild(
    common::Structure & structure,
    const StructureContents & intendedContents,
    const RadiusCalculator & radiusCalculator = RadiusCalculator()
  );

  common::Structure & getStructure();
  const common::Structure & getStructure() const;

  BuildAtomInfo * getAtomInfo(common::Atom & atom);
  const BuildAtomInfo * getAtomInfo(common::Atom & atom) const;
  void insertAtomInfo(BuildAtomInfo & atomInfo);

  ::arma::vec3 getRandomPoint() const;

  double getClusterRadius() const;
  void setClusterRadius(const RadiusCalculator & radiusCalculator);

  FixedSet getFixedSet() const;

  bool extrudeAtoms();

private:

  typedef ::std::map<common::Atom *, BuildAtomInfo> AtomInfoMap;

  common::Structure & myStructure;
  const StructureContents & myIntendedContents;
  AtomInfoMap myAtomsInfo;
  double myClusterRadius;
  AtomExtruder myAtomsExtruder;
};

}
}

#endif /* STRUCTURE_BUILD_H */
