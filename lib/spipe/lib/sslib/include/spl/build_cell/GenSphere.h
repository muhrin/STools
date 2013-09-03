/*
 * GenSphere.h
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef GEN_SPHERE_H
#define GEN_SPHERE_H

// INCLUDES /////////////////////////////////
#include "spl/SSLib.h"

#include <boost/optional.hpp>

#include "spl/build_cell/IGeneratorShape.h"

// FORWARD DECLARES //////////////////////////

namespace spl {
namespace build_cell {

class GenSphere : public IGeneratorShape
{
public:
  typedef ::boost::optional<double> ShellThickness;

  GenSphere(const double radius);
  GenSphere(const GenSphere & toCopy);

  ::arma::vec3 getPosition() const;
  void setPosition(const ::arma::vec3 & pos);

  const ShellThickness & getShellThickness() const;
  void setShellThickness(const ShellThickness thickness);

  // From IGeneratorShape
  virtual ::arma::vec3 randomPoint(const ::arma::mat44 * const transform = NULL) const;
  virtual void randomPoints(
    ::std::vector< ::arma::vec3> & pointsOut,
    const unsigned int num,
    const ::arma::mat44 * const transform = NULL
  ) const;
  virtual OptionalArmaVec3 randomPointOnAxis(const ::arma::vec3 & axis, const ::arma::mat44 * const transform = NULL) const;
  virtual OptionalArmaVec3 randomPointInPlane(const ::arma::vec3 & a, const ::arma::vec3 & b, const ::arma::mat44 * const transform = NULL) const;

  virtual const ::arma::mat44 & getTransform() const;
  virtual void setTransform(const ::arma::mat44 & transform);

  virtual UniquePtr<IGeneratorShape>::Type clone() const;
  // End from IGeneratorShape

private:
  double generateRadius() const;
  ::arma::vec3 randomPoint(const ::arma::mat44 & fullTransform) const;
  void randomPoints(
    ::std::vector< ::arma::vec3> & pointsOut,
    const unsigned int num,
    const ::arma::mat44 & fullTransform
  ) const;
  OptionalArmaVec3 randomPointOnAxis(const ::arma::vec3 & axis, const ::arma::mat44 & fullTransform) const;
  OptionalArmaVec3 randomPointInPlane(const ::arma::vec3 & a, const ::arma::vec3 & b, const ::arma::mat44 & fullTransform) const;

  ::arma::mat44 myTransform;
  double myRadius;
  ShellThickness myShellThickness;
};

}
}

#endif /* GEN_SPHERE_H */
