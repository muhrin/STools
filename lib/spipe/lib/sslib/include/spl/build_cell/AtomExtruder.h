/*
 * AtomExtruder.h
 * TODO: Change name to atoms extruder
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef ATOM_EXTRUDER_H
#define ATOM_EXTRUDER_H

// INCLUDES ////////////
#include "spl/SSLib.h"

#include <set>
#include <vector>

#include <armadillo>

#include "spl/common/Types.h"

namespace spl {
namespace common {
class DistanceCalculator;
class Structure;
}
  
namespace build_cell {

class AtomExtruder
{
public:

  typedef ::std::set<size_t> FixedAtoms;

  static const double DEFAULT_TOLERANCE;
  static const int DEFAULT_MAX_ITERATIONS;

  AtomExtruder();

  bool extrudeAtoms(
    common::Structure & structure) const;

  bool extrudeAtoms(
    common::Structure & structure,
    const FixedAtoms & fixed) const;

  bool extrudeAtoms(
    common::Structure & structure,
    const ::arma::mat & sepSqMtx,
    const FixedAtoms & fixed
  ) const;

  double getTolerance() const;
  void setTolerance(const double tolerance);

  int getMaxIterations() const;
  void setMaxIterations(const int maxIterations);

private:
  typedef ::std::vector<common::Atom *> Atoms;
  typedef ::std::vector<bool> FixedList;

  bool extrude(
    const common::DistanceCalculator & distanceCalc,
    Atoms & atoms,
    const FixedList & fixedList,
    const ::arma::mat & sepSqMtx) const;

  double calcMaxOverlapFractionSq(
    const common::DistanceCalculator & distanceCalc,
    const ::std::vector<common::Atom *> & atoms,
    const FixedList & fixedList,
    const ::arma::mat & sepSqMtx) const;
    
  double myTolerance;
  int myMaxIterations;
};

}
}

#endif /* ATOM_EXTRUDER_H */
