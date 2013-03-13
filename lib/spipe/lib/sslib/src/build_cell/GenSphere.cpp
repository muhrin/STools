/*
 * GenSphere.cpp
 *
 *  Created on: Nov 10, 2011
 *      Author: Martin Uhrin
 */

// INCLUEDES /////////////
#include "build_cell/GenSphere.h"


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
  // TODO: Finish this
  if(myShellThickness)
  {

  }
  ::arma::vec3 point;
  point.randu();

  point += myPosition;
  return point;
}

void GenSphere::randomPoints(::std::vector< ::arma::vec3> & pointsOut, const unsigned int num) const
{
  for(unsigned int i = 0; i < num; ++i)
    pointsOut.push_back(randomPoint());
}

UniquePtr<IGeneratorShape>::Type GenSphere::clone() const
{
  return UniquePtr<IGeneratorShape>::Type(new GenSphere(*this)); 
}

}
}
