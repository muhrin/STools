/*
 * IGeneratorShape.h
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef I_GENERATOR_SHAPE_H
#define I_GENERATOR_SHAPE_H

// INCLUDES ////////////
#include "SSLib.h"

#include <vector>

#include <armadillo>

// DEFINITION ///////////////////////

namespace sstbx {
namespace build_cell {
// FORWARD DECLARATIONS ///////


class IGeneratorShape
{
public:
  virtual ::arma::vec3 randomPoint() const = 0;
  virtual void randomPoints(::std::vector< ::arma::vec3> & pointsOut, const unsigned int num) const = 0;
  virtual UniquePtr<IGeneratorShape>::Type clone() const = 0;
};

}
}


#endif /* I_GENERATOR_SHAPE_H */
