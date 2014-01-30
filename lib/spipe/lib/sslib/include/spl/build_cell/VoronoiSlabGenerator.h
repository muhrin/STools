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

#include <map>
#include <vector>
#include <set>

#include <boost/foreach.hpp>
#include <boost/ptr_container/ptr_vector.hpp>

#include <armadillo>

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Voronoi_diagram_2.h>
#include <CGAL/Triangulation_vertex_base_with_info_2.h>
#include <CGAL/Triangulation_data_structure_2.h>
#include <CGAL/Delaunay_triangulation_adaptation_traits_2.h>
#include <CGAL/Delaunay_triangulation_adaptation_policies_2.h>
#include <CGAL/Periodic_2_triangulation_vertex_base_2.h>
#include <CGAL/Periodic_2_triangulation_traits_2.h>
#include <CGAL/Periodic_2_Delaunay_triangulation_2.h>
#include <CGAL/Polygon_2.h>

#include "spl/SSLibAssert.h"
#include "spl/build_cell/StructureGenerator.h"
#include "spl/common/UnitCell.h"
#include "spl/utility/SharedHandle.h"

// FORWARD DECLARES //////////////////////////

namespace spl {
namespace common {
class DistanceCalculator;
}
namespace build_cell {

class VoronoiSlabGenerator : public StructureGenerator
{
public:
  class Slab;
  class SlabRegion;
  static const int MAX_ITERATIONS;

  VoronoiSlabGenerator()
  {
  }
  VoronoiSlabGenerator(const common::UnitCell & unitCell);

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
  boost::optional< common::UnitCell> myUnitCell;
  std::vector< Slab> mySlabs;
};

class VoronoiSlabGenerator::Slab
{
public:
  struct SiteInfo
  {
    bool fixed;
    boost::optional< double> minsep;
    const VoronoiSlabGenerator::SlabRegion * generatedBy;
  };

  typedef CGAL::Exact_predicates_exact_constructions_kernel K;

private:
  typedef CGAL::Triangulation_vertex_base_with_info_2< SiteInfo, K> Vb;
  typedef CGAL::Triangulation_data_structure_2< Vb> Tds;

  typedef CGAL::Periodic_2_triangulation_vertex_base_2< K, Vb> PVb;
  typedef CGAL::Triangulation_data_structure_2< PVb,
      CGAL::Periodic_2_triangulation_face_base_2< K> > PTds;
  typedef CGAL::Periodic_2_triangulation_traits_2< K> GT;

public:
  typedef CGAL::Delaunay_triangulation_2< K, Tds> Delaunay;
  //typedef CGAL::Periodic_2_Delaunay_triangulation_2< GT, PTds> Delaunay;

private:
//  typedef CGAL::Delaunay_triangulation_adaptation_traits_2< Delaunay> AT;
//  typedef CGAL::Delaunay_triangulation_caching_degeneracy_removal_policy_2<
//      Delaunay> AP;
  typedef utility::SharedHandle< size_t> RegionTicket;
  typedef std::map< const VoronoiSlabGenerator::SlabRegion *, RegionTicket> RegionTickets;

public:
  //typedef CGAL::Voronoi_diagram_2< Delaunay, AT, AP> Voronoi;

  Slab();
  Slab(const arma::mat44 & transform);

  bool
  generateSlab(common::Structure * const structure) const;
  void
  addRegion(UniquePtr< SlabRegion>::Type & region);
  void
  setSaveTriangulation(const bool save);
private:
  typedef boost::ptr_vector< VoronoiSlabGenerator::SlabRegion> SlabRegions;
  typedef std::map< const Delaunay::Vertex_handle, bool> VertexInfo;

  bool
  refineTriangulation(const common::DistanceCalculator & distCalc,
      const common::UnitCell * const unitCell, const RegionTickets & tickets,
      Delaunay * const dg) const;
  bool
  separatePoints(const common::DistanceCalculator & distCalc,
      const common::UnitCell * const unitCell, Delaunay * const dg) const;
  void
  reduceEdges(Delaunay * const dg) const;
  void
  generateAtoms(const Delaunay & dg, common::Structure * const structure) const;

  arma::mat44 myTransform;
  SlabRegions myRegions;
  bool mySaveTriangulation;
};

class VoronoiSlabGenerator::SlabRegion
{
protected:
  typedef CGAL::Exact_predicates_exact_constructions_kernel K;
  typedef CGAL::Polygon_2< K> Boundary;
  typedef std::pair< arma::vec2, bool> Site;

public:
  typedef VoronoiSlabGenerator::Slab::Delaunay Delaunay;
  typedef utility::SharedHandle< size_t> Ticket;
  class Basis;

  SlabRegion(const std::vector< arma::vec2> & boundary,
      UniquePtr< Basis>::Type & basis);
  SlabRegion(const SlabRegion & toCopy);
  virtual
  ~SlabRegion()
  {
  }

  const Boundary &
  getBoundary() const;

  bool
  withinBoundary(const arma::vec2 & r) const;
  bool
  withinBoundary(const Delaunay::Point & r) const;

  virtual Ticket
  generateSites(Delaunay * const dg) const = 0;
  virtual bool
  refine(const Ticket & ticket,
      const std::set< Delaunay::Vertex_handle> & vertices,
      Delaunay * const dg) const
  {
    return true;
  }
  virtual bool
  good(const Ticket & ticket,
      const std::set< Delaunay::Vertex_handle> & vertices,
      const Delaunay & dg) const
  {
    return true;
  }
  virtual std::auto_ptr< SlabRegion>
  clone() const = 0;

  void
  generateAtoms(const Delaunay & dg,
      const std::set< Delaunay::Face_handle> & faces,
      std::vector< common::Atom> * const atoms) const;
private:

  Boundary myBoundary;
  UniquePtr< Basis>::Type myBasis;
};

class VoronoiSlabGenerator::SlabRegion::Basis
{
public:
  virtual
  ~Basis()
  {
  }

  virtual bool
  generateAtoms(const Delaunay & dg,
      const std::set< Delaunay::Face_handle> & faces,
      std::vector< common::Atom> * const atoms) const = 0;
  virtual UniquePtr< Basis>::Type
  clone() const = 0;
};

namespace detail {
template< typename DG>
  bool
  isBoundaryVertex(const DG & dg, const typename DG::Vertex_handle vtx)
  {
    //const typename DG::Vertex_circulator start = dg.incident_vertices(vtx);
    const typename DG::Vertex_circulator start = dg.incident_vertices(vtx);
    if(start.is_empty())
      return false;

    typename DG::Vertex_circulator cl = start;
    do
    {
      if(dg.is_infinite(cl))
        return true;
      ++cl;
    }
    while(cl != start);

    return false;
  }

} // namespace detail

}
}

#endif // SPL_WITH_CGAL
#endif /* VORONOI_SLAB_GENERATOR_H */
