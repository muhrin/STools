/*
 * OptionalTypes.h
 *
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef OPTIONAL_TYPES_H
#define OPTIONAL_TYPES_H

// INCLUDES /////////////////////////////////////////////
#include <string>

#include <boost/optional.hpp>

#include <armadillo>


// FORWARD DECLARATIONS ////////////////////////////////////


namespace sstbx {

typedef ::boost::optional<double> OptionalDouble;
typedef ::boost::optional< ::std::string> OptionalString;
typedef ::boost::optional<unsigned int> OptionalUInt;
typedef ::boost::optional< ::arma::vec3> OptionalVec3;

}

#endif /* OPTIONAL_TYPES_H */
