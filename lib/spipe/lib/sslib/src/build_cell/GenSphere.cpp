/*
 * GenSphere.cpp
 *
 *  Created on: Nov 10, 2011
 *      Author: Martin Uhrin
 */

// INCLUEDES /////////////
#include "build_cell/GenSphere.h"

#include "common/Constants.h"
#include "math/Random.h"

namespace sstbx {
namespace build_cell {

GenSphere::GenSphere(const double radius):
myRadius(abs(radius)),
myPosition(::arma::zeros< ::arma::vec>(3))
{}

GenSphere::GenSphere(const GenSphere & toCopy):
myPosition(toCopy.myPosition),
myRadius(toCopy.myRadius),
myShellThickness(toCopy.myShellThickness)
{}

const ::arma::vec3 & GenSphere::getPosition() const
{
  return myPosition;
}

void GenSphere::setPosition(const ::arma::vec3 & pos)
{
  myPosition = pos;
}

const GenSphere::ShellThickness & GenSphere::getShellThickness() const
{
  return myShellThickness;
}

void GenSphere::setShellThickness(const ShellThickness thickness)
{
  myShellThickness = thickness;
}

::arma::vec3 GenSphere::randomPoint() const
{
  // Get a random point with normally distributed x, y and z with with 0 mean and 1 variance.
  const ::arma::vec3 point(::arma::randn< ::arma::vec>(3));
  
  // Normalise, scale and translate the point
  return point * generateRadius() / (sqrt(::arma::dot(point, point))) + myPosition;
}

void GenSphere::randomPoints(::std::vector< ::arma::vec3> & pointsOut, const unsigned int num) const
{
  for(unsigned int i = 0; i < num; ++i)
    pointsOut.push_back(randomPoint());
}

OptionalArmaVec3 GenSphere::randomPointOnAxis(const ::arma::vec3 & axis) const
{
  // TODO: Axis/sphere intersection test
  return ::arma::vec3(generateRadius() * axis + myPosition);
}

OptionalArmaVec3 GenSphere::randomPointInPlane(const ::arma::vec3 & a, const ::arma::vec3 & b) const
{
  // TODO: Do proper stats for a 2d circle
  // TODO: Plane/sphere intersection test
  const ::arma::vec3 point(math::randn<double>() * a + math::randn<double>() * b);
  // Normalise, scale and translate the point
  return point * generateRadius() / (sqrt(::arma::dot(point, point))) + myPosition;
}

UniquePtr<IGeneratorShape>::Type GenSphere::clone() const
{
  return UniquePtr<IGeneratorShape>::Type(new GenSphere(*this)); 
}

double GenSphere::generateRadius() const
{
  if(myShellThickness)
  {
    const double halfThickness = *myShellThickness * 0.5;
    return myRadius - halfThickness + *myShellThickness * pow(math::randu<double>(), common::constants::ONE_THIRD);
    //return math::randu(myRadius - halfThickness, myRadius + halfThickness)
//      * pow(math::randu<double>(), common::constants::ONE_THIRD);
  }
  else
    return myRadius * pow(math::randu<double>(), common::constants::ONE_THIRD);
}

}
}
