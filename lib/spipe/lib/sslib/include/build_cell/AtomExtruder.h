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
#include "SSLib.h"

#include <set>
#include <vector>

#include <armadillo>

#include "common/Types.h"

namespace sstbx {
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
  static const size_t DEFAULT_MAX_ITERATIONS;

  bool extrudeAtoms(
    common::Structure & structure,
    const size_t maxIterations = DEFAULT_MAX_ITERATIONS,
    const double tolerance = DEFAULT_TOLERANCE) const;

  bool extrudeAtoms(
    common::Structure & structure,
    const FixedAtoms & fixed,
    const size_t maxIterations = DEFAULT_MAX_ITERATIONS,
    const double tolerance = DEFAULT_TOLERANCE) const;

private:
  typedef ::std::vector<common::Atom *> Atoms;
  typedef ::std::vector<bool> FixedList;

  bool extrudeAtoms(
    const common::DistanceCalculator & distanceCalc,
    Atoms & atoms,
    const FixedList & fixedList,
    const ::arma::mat & sepSqMtx,
    const double tolerance,
    const size_t maxIterations) const;

  double calcMaxOverlapFractionSq(
    const common::DistanceCalculator & distanceCalc,
    const ::std::vector<common::Atom *> & atoms,
    const FixedList & fixedList,
    const ::arma::mat & sepSqMtx) const;
    
};

}
}

#endif /* ATOM_EXTRUDER_H */
