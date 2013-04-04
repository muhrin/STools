/*
 * GenBox.cpp
 *
 *  Created on: Nov 10, 2011
 *      Author: Martin Uhrin
 */

// INCLUEDES /////////////
#include "build_cell/GenBox.h"

#include "math/Random.h"
#include "utility/IndexingEnums.h"

namespace sstbx {
namespace build_cell {

GenBox::GenBox(const double width, const double height, const double depth):
myPosition(::arma::zeros< ::arma::vec>(3))
{
  setWidth(width);
  setHeight(height);
  setDepth(depth);
}

GenBox::GenBox(const GenBox & toCopy):
myPosition(toCopy.myPosition),
myShellThickness(toCopy.myShellThickness)
{
  setWidth(toCopy.myWidth);
  setHeight(toCopy.myHeight);
  setDepth(toCopy.myDepth);
}

const ::arma::vec3 & GenBox::getPosition() const
{
  return myPosition;
}

void GenBox::setPosition(const ::arma::vec3 & pos)
{
  myPosition = pos;
}

const GenBox::ShellThickness & GenBox::getShellThickness() const
{
  return myShellThickness;
}

void GenBox::setShellThickness(const ShellThickness thickness)
{
  myShellThickness = thickness;
}

/**
/* To get the shell use rejection algorithm based on flattening a box of a certain
/* thickness and finding points on that:
/*    -
/*   |s|
/*  - - - -
/* |s|b|s|t|
/*  - - - -
/*   |s|
/*    -
/**/
::arma::vec3 GenBox::randomPoint() const
{
  using namespace utility::cart_coords_enum;

  ::arma::vec3 point;
  // TODO: Finish this
  //if(myShellThickness)
  //{
  //  point(Y) = math::rand<double>(*myShellThickness);
  //  do
  //  {
  //    point(X) = math::randu(-myHalfWidth - myHeight, myHalfWidth + myHeight + myWidth);
  //    point(Z) = math::randu(-myHalfDepth - myHeight, myHalfDepth + myHeight);
  //  } while(!isInShell(point))
  //}
  //else // Standard box
  {
    point(X) = math::randu(-myHalfWidth, myHalfWidth);
    point(Y) = math::randu(-myHalfHeight, myHalfHeight);
    point(Z) = math::randu(-myHalfDepth, myHalfDepth);
  }
  // translate
  point += myPosition;
  return point;
}

OptionalArmaVec3 GenBox::randomPointOnAxis(const ::arma::vec3 & axis) const
{
  // TODO: Intersection test of line and box
  const ::arma::vec3 point(randomPoint());
  return OptionalArmaVec3(::arma::dot(point, axis) * axis);
}

OptionalArmaVec3 GenBox::randomPointInPlane(const ::arma::vec3 & a, const ::arma::vec3 & b) const
{
  // TODO: Intersection test of plane and box
  const ::arma::vec3 normal(::arma::cross(a, b)), point(randomPoint());
  return OptionalArmaVec3(point - ::arma::dot(normal, point) * normal);
}

void GenBox::setWidth(const double width)
{
  myWidth = abs(width);
  myHalfWidth = 0.5 * myWidth;
}

void GenBox::setHeight(const double height)
{
  myHeight = abs(height);
  myHalfHeight = 0.5 * myHeight;
}

void GenBox::setDepth(const double depth)
{
  myDepth = abs(depth);
  myHalfDepth = 0.5 * myDepth;
}

void GenBox::randomPoints(::std::vector< ::arma::vec3> & pointsOut, const unsigned int num) const
{
  for(unsigned int i = 0; i < num; ++i)
    pointsOut.push_back(randomPoint());
}

bool GenBox::isInShell(const ::arma::vec3 & point) const
{
  using namespace utility::cart_coords_enum;

  if(point(X) > myHalfWidth)
  {
    // Could be at the right size or the top
    if(point(X) > myHalfWidth + myHeight)
    {
      if(point(Z) > -myHalfDepth && point(X) < myHalfDepth)
      {
        // It's on the top

      }
    }
  }
  else if(point(X) < myHalfWidth)
  {
    if(point(Z) < -myHalfDepth || point(Z) > myHalfDepth)
      return false;
    else
      return true;
  }
  return false;
}

UniquePtr<IGeneratorShape>::Type GenBox::clone() const
{
  return UniquePtr<IGeneratorShape>::Type(new GenBox(*this));
}

}
}
