/*
 * GenBox.h
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef GEN_BOX_H
#define GEN_BOX_H

// INCLUDES /////////////////////////////////
#include "SSLib.h"

#include <boost/optional.hpp>

#include "build_cell/IGeneratorShape.h"

// FORWARD DECLARES //////////////////////////

namespace sstbx {
namespace build_cell {

class GenBox : public IGeneratorShape
{
public:
  typedef ::boost::optional<double> ShellThickness;

  GenBox(const double width, const double height, const double depth);
  GenBox(const GenBox & toCopy);

  const ::arma::vec3 & getPosition() const;
  void setPosition(const ::arma::vec3 & pos);

  const ShellThickness & getShellThickness() const;
  void setShellThickness(const ShellThickness thickness);

  // From IGeneratorShape
  virtual ::arma::vec3 randomPoint() const;
  virtual void randomPoints(::std::vector< ::arma::vec3> & pointsOut, const unsigned int num) const;

  // Generate a random point on an axis that conforms to the shape.  Axis is assumed to be normalised.
  virtual OptionalArmaVec3 randomPointOnAxis(const ::arma::vec3 & axis) const;
  // Generate a random point in a plane that conforms to the shape.
  virtual OptionalArmaVec3 randomPointInPlane(const ::arma::vec3 & a, const ::arma::vec3 & b) const; 

  virtual UniquePtr<IGeneratorShape>::Type clone() const;
  // End from IGeneratorShape

  void setWidth(const double width);
  void setHeight(const double height);
  void setDepth(const double depth);

private:

  bool isInShell(const ::arma::vec3 & point) const;

  double myWidth, myHalfWidth;
  double myHeight, myHalfHeight;
  double myDepth, myHalfDepth;


  ::arma::vec3 myPosition;
  ShellThickness myShellThickness;
};

}
}

#endif /* GEN_BOX_H */
