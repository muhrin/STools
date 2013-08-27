/*
 * OptimisationSettings.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "spl/potential/OptimisationSettings.h"

#include <boost/foreach.hpp>

// NAMESPACES ////////////////////////////////


namespace spl {
namespace potential {

void
OptimisationSettings::insertConstraint(OptimisationConstraint & constraint)
{
  myConstraints.push_back(constraint.clone().release());
}

OptimisationSettings::ConstraintsIterator
OptimisationSettings::beginConstraints()
{
  return myConstraints.begin();
}

OptimisationSettings::ConstraintsIterator
OptimisationSettings::endConstraints()
{
  return myConstraints.end();
}

OptimisationSettings::ConstraintsIterator
OptimisationSettings::eraseConstraint(ConstraintsIterator it)
{
  return myConstraints.erase(it);
}

void OptimisationSettings::applyLatticeConstraints(
  const common::Structure & structure,
  const ::arma::mat33 & lattice,
  ::arma::mat33 & deltaLattice) const
{
  BOOST_FOREACH(const OptimisationConstraint & constraint, myConstraints)
  {
    constraint.constrainLattice(structure, lattice, deltaLattice);
  }
}

void OptimisationSettings::applyAtomsConstraints(
  const common::Structure & structure,
  const ::arma::mat & currentPos,
  ::arma::mat & deltaPos) const
{
  BOOST_FOREACH(const OptimisationConstraint & constraint, myConstraints)
  {
    constraint.constrainAtoms(structure, currentPos, deltaPos);
  }
}

}
}

