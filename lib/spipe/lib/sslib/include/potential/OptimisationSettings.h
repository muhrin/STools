/*
 * OptimisationSettings.h
 *
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef OPTIMISATION_SETTINGS_H
#define OPTIMISATION_SETTINGS_H

// INCLUDES /////////////////////////////////////////////

#include <vector>

#include <boost/ptr_container/ptr_list.hpp>

#include <armadillo>

#include <OptionalTypes.h>

#include "common/Types.h"
#include "potential/OptimisationConstraint.h"

// DEFINES //////////////////////////////////////////////

namespace sstbx {
namespace potential {

// FORWARD DECLARATIONS ////////////////////////////////////
class OptimisationConstraint;

struct OptimisationSettings
{
  typedef ::boost::ptr_list<OptimisationConstraint> Constraints;
  typedef Constraints::iterator ConstraintsIterator;
  typedef Constraints::const_iterator ConstConstraintsIterator;

  struct Optimise
  {
    enum Value
    {
      ATOMS             = 0x01, // 000
      LATTICE           = 0x02, // 010
      ATOMS_AND_LATTICE = 0x03  // 011
    };
  };

  ::sstbx::OptionalUInt maxSteps;  // Max geom-optimisation steps
  ::sstbx::OptionalArmaMat33 pressure;
  ::boost::optional<Optimise::Value> optimisationType;

  void insertConstraint(OptimisationConstraint & constraint);
  ConstraintsIterator beginConstraints();
  ConstraintsIterator endConstraints();
  ConstraintsIterator eraseConstraint(ConstraintsIterator it);

  void applyLatticeConstraints(
    const common::Structure & structure,
    const ::arma::mat33 & lattice,
    ::arma::mat33 & deltaLattice) const;

  void applyAtomsConstraints(
    const common::Structure & structure,
    const ::arma::mat & currentPos,
    ::arma::mat & deltaPos) const;

private:

  Constraints myConstraints;
};


}
}

#endif /* OPTIMISATION_SETTINGS_H */
