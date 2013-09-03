/*
 * IGeneratorShape.h
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef I_GENERATOR_SHAPE_H
#define I_GENERATOR_SHAPE_H

// INCLUDES ////////////
#include "spl/SSLib.h"

#include <vector>

#include <armadillo>

#include "spl/OptionalTypes.h"

// DEFINITION ///////////////////////

namespace spl {
namespace build_cell {
// FORWARD DECLARATIONS ///////


class IGeneratorShape
{
public:

  virtual ::arma::vec3 randomPoint(const ::arma::mat44 * const transform = NULL) const = 0;
  virtual void randomPoints(
    ::std::vector< ::arma::vec3> & pointsOut,
    const unsigned int num,
    const ::arma::mat44 * const transform = NULL
  ) const = 0;
  // Generate a random point on an axis that conforms to the shape (if possible).  Axis is assumed to be normalised.
  virtual OptionalArmaVec3 randomPointOnAxis(const ::arma::vec3 & axis, const ::arma::mat44 * const transform = NULL) const = 0;
  // Generate a random point in a plane that conforms to the shape (if possible).
  virtual OptionalArmaVec3 randomPointInPlane(const ::arma::vec3 & a, const ::arma::vec3 & b, const ::arma::mat44 * const transform = NULL) const = 0;

  virtual const ::arma::mat44 & getTransform() const = 0;
  virtual void setTransform(const ::arma::mat44 & transform) = 0;
 
  virtual UniquePtr<IGeneratorShape>::Type clone() const = 0;
};

}
}


#endif /* I_GENERATOR_SHAPE_H */
