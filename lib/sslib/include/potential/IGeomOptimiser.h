/*
 * IGeomOptimiser.h
 *
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef I_GEOM_OPTIMISER_H
#define I_GEOM_OPTIMISER_H

// INCLUDES /////////////////////////////////////////////

#include <boost/shared_ptr.hpp>

#include <armadillo>

// DEFINES //////////////////////////////////////////////


// FORWARD DECLARATIONS ////////////////////////////////////
namespace sstbx
{
namespace common
{
class Structure;
}
namespace potential
{
template <typename FloatType>
struct StandardData;
class IPotential;
}
}

namespace sstbx
{
namespace potential
{

class IGeomOptimiser
{
public:

	virtual ~IGeomOptimiser() {}

  /**
  /* Get the potential being used by the geometry optimiser.  Not all
  /* geometry optimisers need to have a potential in which case NULL
  /* will be returned.
  /**/
  virtual const IPotential * getPotential() const = 0;

  // TODO: Allow return value to give some indication of the reason for the failure!
	virtual bool optimise(
    ::sstbx::common::Structure & structure,
    const ::arma::mat * const externalPressure = NULL) const = 0;

	virtual bool optimise(
		::sstbx::common::Structure & structure,
    ::boost::shared_ptr<StandardData<double> > & data,
    const ::arma::mat * const externalPressure = NULL) const = 0;
};

}
}

#endif /* I_GEOM_OPTIMISER_H */
