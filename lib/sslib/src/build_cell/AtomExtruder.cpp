/*
 * AtomExtruder.cpp
 *
 *  Created on: Nov 10, 2011
 *      Author: Martin Uhrin
 */

#include "build_cell/AtomExtruder.h"

#include <boost/multi_array.hpp>

#include "common/Atom.h"
#include "common/AbstractFmidCell.h"
#include "common/Structure.h"
#include "common/Types.h"

namespace sstbx {
namespace build_cell {

bool AtomExtruder::extrudeAtoms(common::Structure & structure) const
{
  const ::std::vector<common::AtomPtr> & atoms = structure.getAtoms();

  ::std::vector<common::AtomPtr> atomsWithRadii;

  for(size_t i = 0; i < atoms.size(); ++i)
  {
    if(atoms[i]->getRadius() != 0.0)
    {
      atomsWithRadii.push_back(atoms[i]);
    }
  }

  // Calculate seaparation matrix
  ::arma::mat sepSqMtx(atomsWithRadii.size(), atomsWithRadii.size());


  double radius1;
  for(size_t row = 0; row < atomsWithRadii.size(); ++row)
  {
    radius1 = atomsWithRadii[row]->getRadius();
    for(size_t col = row; col < atomsWithRadii.size(); ++col)
    {
      sepSqMtx(row, col) = radius1 + atomsWithRadii[col]->getRadius();
      sepSqMtx(row, col) *= sepSqMtx(row, col);
    }
  }

  return extrudeAtoms(*structure.getUnitCell(), atomsWithRadii, sepSqMtx);
}

bool AtomExtruder::extrudeAtoms(
  const common::AbstractFmidCell & cell,
  ::std::vector<common::AtomPtr> & atoms,
  const ::arma::mat & sepSqMtx,
  const size_t maxIterations) const
{
  typedef ::boost::multi_array< ::arma::vec, 2> array_type;
  typedef array_type::index index;

  const size_t numAtoms = atoms.size();
  const double tolerance = 0.00001;
  double sep, sepDiff;
  
  array_type sepVectors(::boost::extents[numAtoms][numAtoms]);
  ::arma::mat sepSqDistances(numAtoms, numAtoms);

  double maxOverlapFractionSq;
  ::arma::vec dr; // The displacement vector to move each atom by
  size_t i, j;
  bool success = false;
  for(size_t iters = 0; iters < maxIterations; ++iters)
  {
    // First loop over calculating separations and checking for overlap
    maxOverlapFractionSq = 0.0;
    
    for(i = 0; i < numAtoms - 1; ++i)
    {
      const ::arma::vec & posI = atoms[i]->getPosition();
      for(j = i + 1; j < numAtoms; ++j)
      {
        const ::arma::vec & posJ = atoms[j]->getPosition();

        sepVectors[i][j]     = cell.getVecMinimumImg(posI, posJ);
        sepSqDistances(i, j) = ::arma::dot(sepVectors[i][j], sepVectors[i][j]);

        // Are they closer than the sum of the two radii?
        if(sepSqDistances(i, j) < sepSqMtx(i, j))
        {
          maxOverlapFractionSq = ::std::max(maxOverlapFractionSq, (sepSqMtx(i, j) - sepSqDistances(i, j)) / sepSqMtx(i, j));
        }
      }
    }

    if(maxOverlapFractionSq < tolerance)
    {
      success = true;
      break;
    }


    // Now fix-up any overlaps
    for(size_t i = 0; i < atoms.size() - 1; ++i)
    {
      const ::arma::vec & posI = atoms[i]->getPosition();
      for(size_t j = i + 1; j < atoms.size(); ++j)
      {
        if(sepSqDistances(i, j) < sepSqMtx(i, j))
        {
          sep = sqrt(sepSqDistances(i, j));
          const ::arma::vec & posJ = atoms[j]->getPosition();

          sepDiff = sqrt(sepSqMtx(i, j)) - sep;

          // Generate the displacement vector
          dr = 0.5 * sepDiff / sep * sepVectors[i][j];

          atoms[i]->setPosition(posI - dr);
          atoms[j]->setPosition(posJ + dr);
        }
      }
    }
  }
  return success;
}

}
}
