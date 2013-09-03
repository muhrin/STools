/*
 * AtomExtruder.cpp
 *
 *  Created on: Nov 10, 2011
 *      Author: Martin Uhrin
 */

#include "spl/build_cell/AtomExtruder.h"

#include <boost/multi_array.hpp>

#include "spl/common/Atom.h"
#include "spl/common/Structure.h"
#include "spl/common/DistanceCalculator.h"
#include "spl/common/Types.h"

namespace spl {
namespace build_cell {

const int AtomExtruder::DEFAULT_MAX_ITERATIONS = 7000;
const double AtomExtruder::DEFAULT_TOLERANCE = 0.001;

AtomExtruder::AtomExtruder() :
    myMaxIterations(DEFAULT_MAX_ITERATIONS), myTolerance(DEFAULT_TOLERANCE)
{
}

bool
AtomExtruder::extrudeAtoms(common::Structure & structure) const
{
  return extrudeAtoms(structure, FixedAtoms());
}

bool
AtomExtruder::extrudeAtoms(common::Structure & structure,
    const FixedAtoms & fixed) const
{
  const size_t numAtoms = structure.getNumAtoms();

  Atoms atomsWithRadii;
  FixedList allFixed;

  FixedAtoms fixedRemaining(fixed);
  FixedAtoms::iterator it;
  for(size_t i = 0; i < numAtoms; ++i)
  {
    common::Atom & atom = structure.getAtom(i);
    if(atom.getRadius() != 0.0)
    {
      atomsWithRadii.push_back(&atom);
      it = fixedRemaining.find(i);
      if(it == fixedRemaining.end())
        allFixed.push_back(false);
      else
      {
        allFixed.push_back(true);
        fixedRemaining.erase(it);
      }
    }
  }

  if(atomsWithRadii.size() == 0)
    return true;

  // Calculate seaparation matrix
  ::arma::mat sepSqMtx(atomsWithRadii.size(), atomsWithRadii.size());

  double radius1;
  for(size_t row = 0; row < atomsWithRadii.size() - 1; ++row)
  {
    radius1 = atomsWithRadii[row]->getRadius();
    for(size_t col = row + 1; col < atomsWithRadii.size(); ++col)
    {
      sepSqMtx(row, col) = radius1 + atomsWithRadii[col]->getRadius();
      sepSqMtx(row, col) *= sepSqMtx(row, col);
    }
  }

  return extrude(structure.getDistanceCalculator(), atomsWithRadii,
      allFixed, sepSqMtx);
}

bool
AtomExtruder::extrudeAtoms(common::Structure & structure,
    const ::arma::mat & sepSqMtx, const FixedAtoms & fixed) const
{
  SSLIB_ASSERT(sepSqMtx.is_square());
  SSLIB_ASSERT(sepSqMtx.n_rows == structure.getNumAtoms());

  Atoms atoms;
  for(int i = 0; i < structure.getNumAtoms(); ++i)
    atoms.push_back(&structure.getAtom(i));

  FixedList fixedList(structure.getNumAtoms(), false);
  BOOST_FOREACH(const size_t index, fixed)
    fixedList[index] = true;

  return extrude(structure.getDistanceCalculator(), atoms, fixedList, sepSqMtx);
}

double
AtomExtruder::getTolerance() const
{
  return myTolerance;
}

void
AtomExtruder::setTolerance(const double tolerance)
{
  myTolerance = tolerance;
}

int
AtomExtruder::getMaxIterations() const
{
  return myMaxIterations;
}

void
AtomExtruder::setMaxIterations(const int maxIterations)
{
  myMaxIterations = maxIterations;
}

bool
AtomExtruder::extrude(const common::DistanceCalculator & distanceCalc,
    ::std::vector< common::Atom *> & atoms, const FixedList & fixedList,
    const ::arma::mat & sepSqMtx) const
{
  typedef ::boost::multi_array< ::arma::vec, 2> array_type;
  typedef array_type::index index;

  const int numAtoms = static_cast< int>(atoms.size());
  if(numAtoms == 0)
    return true;

  const double toleranceSq = myTolerance * myTolerance;
  double sep, sepDiff;

  //array_type sepVectors(::boost::extents[numAtoms][numAtoms]);
  //::arma::mat sepSqDistances(numAtoms, numAtoms);

  double maxOverlapFractionSq;

  double prefactor; // Used to adjust the displacement vector if either atom is fixed
  double sepSq;
  ::arma::vec3 dr, sepVec;
  int row, col;
  bool success = false;

  for(size_t iters = 0; iters < myMaxIterations; ++iters)
  {
    // First loop over calculating separations and checking for overlap
    maxOverlapFractionSq = calcMaxOverlapFractionSq(distanceCalc, atoms,
        fixedList, sepSqMtx);

    if(maxOverlapFractionSq < toleranceSq)
    {
      success = true;
      break;
    }

    // Now fix-up any overlaps
    for(row = 0; row < numAtoms - 1; ++row)
    {
      const ::arma::vec & posI = atoms[row]->getPosition();
      for(col = row + 1; col < numAtoms; ++col)
      {
        if(fixedList[row] && fixedList[col])
          continue; // Both fixed, don't move

        const ::arma::vec & posJ = atoms[col]->getPosition();
        sepVec = distanceCalc.getVecMinImg(posI, posJ);
        sepSq = ::arma::dot(sepVec, sepVec);
        if(sepSq < sepSqMtx(row, col))
        {
          if(fixedList[row] || fixedList[col])
            prefactor = 1.0; // Only one fixed, displace it by the full amount
          else
            prefactor = 0.5; // None fixed, share displacement equally

          sep = sqrt(sepSq);
          sepDiff = sqrt(sepSqMtx(row, col)) - sep;

          // Generate the displacement vector
          dr = prefactor * sepDiff / sep * sepVec;

          if(!fixedList[row])
            atoms[row]->setPosition(posI - dr);
          if(!fixedList[col])
            atoms[col]->setPosition(posJ + dr);
        }
      }
    }
  }
  return success;
}

double
AtomExtruder::calcMaxOverlapFractionSq(
    const common::DistanceCalculator & distanceCalc,
    const ::std::vector< common::Atom *> & atoms, const FixedList & fixedList,
    const ::arma::mat & sepSqMtx) const
{
  const int numAtoms = static_cast< int>(sepSqMtx.n_rows);

  int row, col;
  double sepSq, maxOverlapFractionSq = 0.0;
  for(row = 0; row < numAtoms - 1; ++row)
  {
    const ::arma::vec & posI = atoms[row]->getPosition();
    for(col = row + 1; col < numAtoms; ++col)
    {
      if(fixedList[row] && fixedList[col])
        continue; // Ignore case where both atoms are fixed

      const ::arma::vec & posJ = atoms[col]->getPosition();

      sepSq = distanceCalc.getDistSqMinImg(posI, posJ);
      // Are they closer than the sum of the two radii?
      if(sepSq < sepSqMtx(row, col))
      {
        maxOverlapFractionSq = ::std::max(maxOverlapFractionSq,
            ::std::abs(sepSqMtx(row, col) - sepSq) / sepSqMtx(row, col));
      }
    }
  }
  return maxOverlapFractionSq;
}

}
}
