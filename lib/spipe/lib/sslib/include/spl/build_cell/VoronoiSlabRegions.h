/*
 * VoronoiSlabRegions.h
 *
 *  Created on: Jan 17, 2014
 *      Author: Martin Uhrin
 */

#ifndef VORONOI_SLAB_REGIONS_H
#define VORONOI_SLAB_REGIONS_H

// INCLUDES /////////////////////////////////
#include "spl/SSLib.h"

#ifdef SPL_WITH_CGAL

#include <vector>

#include <boost/optional.hpp>

#include "spl/build_cell/VoronoiSlabGenerator.h"

#include "spl/utility/Range.h"

// FORWARD DECLARES //////////////////////////

namespace spl {
namespace math {
class Line2;
}
namespace build_cell {

class LatticeRegion : public VoronoiSlabGenerator::SlabRegion
{
public:
  struct ConstructionInfo
  {
    std::vector< arma::vec2> boundary;
    arma::vec2 vecA;
    arma::vec2 vecB;
    boost::optional< arma::vec2> startPoint;
  };

  explicit
  LatticeRegion(const ConstructionInfo & info, UniquePtr< Basis>::Type & basis);
  LatticeRegion(const LatticeRegion & toCopy);

  virtual void
  generateSites(Delaunay * const dg) const;
  virtual std::auto_ptr< SlabRegion>
  clone() const;

private:
  typedef utility::MinMax< int> LatticeMultiples;

  LatticeMultiples
  getLatticeMultiples(const arma::vec2 & x0, const arma::vec2 & latticeVecP,
      const arma::vec2 & latticeVecQ) const;
  int
  getLatticeVecMultiple(const arma::vec2 & x0, const arma::vec2 & latticeVec,
      const math::Line2 & latticeLine,
      const math::Line2 & intersectionLine) const;
  void
  generateLine(const arma::vec2 & r0, const arma::vec2 & dr,
      const LatticeMultiples & range, const CGAL::Bbox_2 & box,
      Delaunay * const dg) const;
  bool
  inBBox(const arma::vec2 & r, const CGAL::Bbox_2 & bbox) const;

  arma::vec2 myLattice[2];
  boost::optional< arma::vec2> myStartPoint;
};

class RandomRegion : public VoronoiSlabGenerator::SlabRegion
{
public:
  RandomRegion(const std::vector< arma::vec2> & boundary, const int numPoints,
      const double minsep, UniquePtr< Basis>::Type & basis);
  virtual void
  generateSites(Delaunay * const dg) const;
  virtual std::auto_ptr< SlabRegion>
  clone() const;
private:
  const int myNumPoints;
  const double myMinsep;
};

class OrderedBasis : public VoronoiSlabGenerator::SlabRegion::Basis
{
public:
  explicit
  OrderedBasis(const std::vector< std::string> & species);

  virtual bool
  generateAtoms(const Voronoi & vd, std::set< Voronoi::Vertex_handle> vertices,
      std::vector< common::Atom> * const atoms) const;

  virtual UniquePtr< Basis>::Type
  clone() const;
private:
  void
  placeAtoms(const size_t basisIdx,
      const std::set< Voronoi::Vertex_handle>::iterator & it,
      std::set< Voronoi::Vertex_handle> * const vertices,
      std::vector< common::Atom> * const atoms) const;

  std::vector< std::string> mySpecies;
};

}
}

#endif // SPL_WITH_CGAL
#endif /* VORONOI_SLAB_REGIONS_H */
