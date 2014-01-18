/*
 * VoronoiSlabGenerator.h
 *
 *  Created on: Jan 17, 2014
 *      Author: Martin Uhrin
 */

#ifndef VORONOI_SLAB_GENERATOR_H
#define VORONOI_SLAB_GENERATOR_H

// INCLUDES /////////////////////////////////
#include "spl/SSLib.h"

#ifdef SPL_WITH_CGAL

#include <vector>
#include <set>

#include <boost/foreach.hpp>
#include <boost/ptr_container/ptr_vector.hpp>

#include <armadillo>

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
//#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Voronoi_diagram_2.h>
#include <CGAL/Delaunay_triangulation_adaptation_traits_2.h>
#include <CGAL/Delaunay_triangulation_adaptation_policies_2.h>
#include <CGAL/Polygon_2.h>

#include "spl/SSLibAssert.h"
#include "spl/build_cell/StructureGenerator.h"
#include "spl/utility/Armadillo.h"

// FORWARD DECLARES //////////////////////////

namespace spl {
namespace build_cell {

class VoronoiSlabGenerator : public StructureGenerator
{
public:
  class Slab;
  class SlabRegion;

  virtual GenerationOutcome
  generateStructure(common::StructurePtr & structureOut,
      const common::AtomSpeciesDatabase & speciesDb);
  virtual GenerationOutcome
  generateStructure(common::StructurePtr & structureOut,
      const common::AtomSpeciesDatabase & speciesDb,
      const GenerationSettings & info);

  void
  addSlab(const Slab & slab);
private:
  std::vector< Slab> mySlabs;
};

class VoronoiSlabGenerator::Slab
{
  typedef CGAL::Exact_predicates_exact_constructions_kernel K;
  typedef CGAL::Delaunay_triangulation_2< K> Delaunay;
  typedef CGAL::Delaunay_triangulation_adaptation_traits_2< Delaunay> AT;
  typedef CGAL::Delaunay_triangulation_caching_degeneracy_removal_policy_2<
      Delaunay> AP;
  typedef std::pair< arma::vec2, bool> Site;
public:
  typedef CGAL::Voronoi_diagram_2< Delaunay, AT, AP> Voronoi;

  Slab();
  Slab(const arma::mat44 & transform);

  bool
  generateSlab(common::Structure * const structure) const;
  void
  addRegion(UniquePtr< SlabRegion>::Type & region);
private:
  typedef boost::ptr_vector< VoronoiSlabGenerator::SlabRegion> SlabRegions;

  arma::mat44 myTransform;
  SlabRegions myRegions;
};

class VoronoiSlabGenerator::SlabRegion
{
protected:
  typedef CGAL::Exact_predicates_exact_constructions_kernel K;
  typedef CGAL::Polygon_2< K> Boundary;
  typedef VoronoiSlabGenerator::Slab::Voronoi Voronoi;
  typedef std::pair< arma::vec2, bool> Site;

public:
  SlabRegion(const std::vector< arma::vec2> & boundary)
  {
    BOOST_FOREACH(const arma::vec2 & r, boundary)
      myBoundary.push_back(utility::toCgalPoint< K>(r));
    SSLIB_ASSERT(!myBoundary.is_empty());
    SSLIB_ASSERT(myBoundary.is_simple());
  }
  virtual
  ~SlabRegion()
  {
  }

  const Boundary &
  getBoundary() const
  {
    return myBoundary;
  }

  bool
  withinBoundary(const arma::vec2 & r) const
  {
    return getBoundary().bounded_side(K::Point_2(r(0), r(1)))
        != CGAL::ON_UNBOUNDED_SIDE;
  }

  virtual void
  generateSites(std::vector< Site> * const points) const = 0;
  virtual void
  generateAtoms(const Voronoi & vd, std::set< Voronoi::Vertex_handle> vertices,
      std::vector< common::Atom> * const atoms) const = 0;
  virtual std::auto_ptr< SlabRegion>
  clone() const = 0;
private:
  Boundary myBoundary;
};

}
}

#endif // SPL_WITH_CGAL
#endif /* VORONOI_SLAB_GENERATOR_H */
