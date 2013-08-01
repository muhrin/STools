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
#include <string>
#include <vector>

#include <boost/ptr_container/ptr_vector.hpp>

#include "build_cell/AtomExtruder.h"
#include "build_cell/BuildAtomInfo.h"
#include "build_cell/IGeneratorShape.h"

namespace sstbx {
namespace common {
class Atom;
class Structure;
}
namespace build_cell {

class BuildAtomInfo;
class StructureContents;
class SymmetryGroup;

class StructureBuild
{
  typedef ::std::map<common::Atom *, BuildAtomInfo *> AtomInfoMap;
  typedef ::boost::ptr_vector<BuildAtomInfo> AtomInfoList;
public:
  typedef ::std::set<size_t> FixedSet;
  typedef UniquePtr<SymmetryGroup>::Type SymmetryGroupPtr;
  typedef AtomInfoList::iterator AtomInfoIterator;
  typedef UniquePtr<IGeneratorShape>::Type GenShapePtr;
  typedef ::std::vector< ::arma::mat44> TransformStack;

  struct SpeciesPairDistances
  {
    ::std::vector< ::std::string> species;
    ::arma::mat distances;
  };

  typedef ::std::vector<SpeciesPairDistances> SpeciesPairDistancesStack;

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
  StructureBuild(
    common::Structure & structure,
    const StructureContents & intendedContents,
    GenShapePtr genShape
  );

  common::Structure & getStructure();
  const common::Structure & getStructure() const;

  BuildAtomInfo * getAtomInfo(common::Atom & atom);
  const BuildAtomInfo * getAtomInfo(common::Atom & atom) const;
  BuildAtomInfo & createAtomInfo(common::Atom & atom);

  size_t getNumAtomInfos() const;
  AtomInfoIterator beginAtomInfo();
  AtomInfoIterator endAtomInfo();
  void addAtom(common::Atom & atom, BuildAtomInfo & atomInfo);
  void removeAtom(common::Atom & atom);

  const IGeneratorShape & getGenShape() const;

  const SymmetryGroup * getSymmetryGroup() const;
  void setSymmetryGroup(SymmetryGroupPtr symGroup);

  FixedSet getFixedSet() const;

  bool extrudeAtoms();

  void pushTransform(const ::arma::mat44 & transform);
  void popTransform();
  const ::arma::mat44 & getTransform() const;

  void pushSpeciesPairDistances(const SpeciesPairDistances & distances);
  void popSpeciesPairDistances();
  const SpeciesPairDistances & getSpeciesPairDistances() const;

private:

  void atomInserted(BuildAtomInfo & atomInfo, common::Atom & atom);
  void atomRemoved(common::Atom & atom);

  common::Structure & myStructure;
  const StructureContents & myIntendedContents;
  AtomInfoMap myAtomsInfo;
  AtomInfoList myAtomInfoList;
  AtomExtruder myAtomsExtruder;
  SymmetryGroupPtr mySymmetryGroup;
  GenShapePtr myGenShape;

  TransformStack myTransformStack;
  mutable ::arma::mat44 myTransform;
  mutable bool myTransformCurrent;

  SpeciesPairDistancesStack mySpeciesPairDistancesStack;
  mutable SpeciesPairDistances mySpeciesPairDistances;
  mutable bool mySpeciesPairDistancesCurrent;

  friend class BuildAtomInfo;
};

}
}

#endif /* STRUCTURE_BUILD_H */
