/*
 * DistanceCalculator.h
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

#ifndef DISTANCE_CALCULATOR_H
#define DISTANCE_CALCULATOR_H

// INCLUDES ///////////////////////////////////
#include <armadillo>

#include "common/Atom.h"

namespace sstbx {
namespace common {

class Structure;

class DistanceCalculator
{
public:

  static const size_t DEFAULT_MAX_OUTPUTS;

  DistanceCalculator(const Structure & structure);
  virtual ~DistanceCalculator() {}

  virtual inline double getDistMinImg(const ::arma::vec3 & a, const ::arma::vec3 & b) const
  { return sqrt(getDistSqMinImg(a, b)); }

  virtual inline double getDistMinImg(const Atom & atom1, const Atom & atom2) const
  { return getDistMinImg(atom1.getPosition(), atom2.getPosition()); }

  virtual inline double getDistSqMinImg(const ::arma::vec3 & a, const ::arma::vec3 & b) const
  {
    const ::arma::vec3 dr = getVecMinImg(a, b);
    return ::arma::dot(dr, dr);
  }

  virtual bool getDistsBetween(
    const ::arma::vec3 & a,
    const ::arma::vec3 & b,
    const double cutoff,
    ::std::vector<double> & outDistances,
    const size_t maxDistances = DEFAULT_MAX_OUTPUTS) const = 0;

  virtual inline bool getDistsBetween(
    const Atom & atom1,
    const Atom & atom2,
    const double cutoff,
    ::std::vector<double> & outDistances,
    const size_t maxDistances = DEFAULT_MAX_OUTPUTS) const
  {
    return getDistsBetween(atom1.getPosition(), atom2.getPosition(), cutoff, outDistances, maxDistances);
  }

  virtual inline double getDistSqMinImg(const Atom & atom1, const Atom & atom2) const
  { return getDistSqMinImg(atom1.getPosition(), atom2.getPosition()); }

  virtual ::arma::vec3 getVecMinImg(const ::arma::vec3 & a, const ::arma::vec3 & b) const = 0;

  virtual ::arma::vec3 getVecMinImg(const Atom & atom1, const Atom & atom2) const
  { return getVecMinImg(atom1.getPosition(), atom2.getPosition()); }

  virtual bool getVecsBetween(
    const ::arma::vec3 & a,
    const ::arma::vec3 & b,
    const double cutoff,
    ::std::vector< ::arma::vec3> & outVectors,
    const size_t maxVectors = DEFAULT_MAX_OUTPUTS) const = 0;

  virtual inline bool getVecsBetween(
    const Atom & atom1,
    const Atom & atom2,
    const double cutoff,
    ::std::vector< ::arma::vec3> & outVectors,
    const size_t maxVectors = DEFAULT_MAX_OUTPUTS) const
  { return getVecsBetween(atom1.getPosition(), atom2.getPosition(), cutoff, outVectors, maxVectors); }

  virtual bool isValid() const = 0;

protected:

  const Structure & myStructure;

};

} // namespace common
} // namespace sstbx

#endif /* DISTANCE_CALCULATOR_H */
