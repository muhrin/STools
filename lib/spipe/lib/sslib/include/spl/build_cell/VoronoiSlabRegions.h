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

#include "spl/build_cell/VoronoiSlabGenerator.h"
// FORWARD DECLARES //////////////////////////

namespace spl {
namespace build_cell {

class LatticeRegion : public VoronoiSlabGenerator::SlabRegion
{
public:
  LatticeRegion(const std::vector< arma::vec2> & boundary,
      const arma::vec2 & vecA, const arma::vec2 & vecB,
      const std::vector< std::string> & basis);
  LatticeRegion(const LatticeRegion & toCopy);

  virtual void
  generateSites(std::vector< Site> * const points) const;
  virtual void
  generateAtoms(const Voronoi & vd, std::set< Voronoi::Vertex_handle> vertices,
      std::vector< common::Atom> * const atoms) const;
  virtual std::auto_ptr< SlabRegion>
  clone() const;

private:
  void
  generateLine(const arma::vec2 & r0, const arma::vec2 & dr,
      const CGAL::Bbox_2 & box, std::vector< Site> * const points) const;
  void
  placeAtoms(const size_t basisIdx,
      const std::set< Voronoi::Vertex_handle>::iterator & it,
      std::set< Voronoi::Vertex_handle> * const vertices,
      std::vector< common::Atom> * const atoms) const;
  bool
  inBBox(const arma::vec2 & r, const CGAL::Bbox_2 & bbox) const;

  arma::vec2 myLattice[2];
  std::vector< std::string> myBasis;
};

}
}

#endif // SPL_WITH_CGAL
#endif /* VORONOI_SLAB_REGIONS_H */
