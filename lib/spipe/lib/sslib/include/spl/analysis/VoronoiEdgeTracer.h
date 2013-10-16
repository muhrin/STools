/*
 * VoronoiEdgeTracer.h
 *
 *  Created on: Oct 15, 2013
 *      Author: Martin Uhrin
 */

#ifndef VORONOI_EDGE_TRACER_H
#define VORONOI_EDGE_TRACER_H

// INCLUDES ////////////
#include "spl/SSLib.h"

#ifdef SSLIB_USE_CGAL

#include <map>
#include <set>

#include <CGAL/Arr_segment_traits_2.h>
#include <CGAL/Arrangement_2.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_adaptation_traits_2.h>
#include <CGAL/Delaunay_triangulation_adaptation_policies_2.h>
#include <CGAL/Triangulation_vertex_base_with_info_2.h>
#include <CGAL/Voronoi_diagram_2.h>

// FORWARD DECLARATIONS ///////

// DEFINITION ///////////////////////

namespace spl {
namespace analysis {

template< typename LabelType>
  class VoronoiEdgeTracer
  {
    typedef CGAL::Exact_predicates_inexact_constructions_kernel K;

    // Arrangements stuff
    typedef CGAL::Arr_segment_traits_2< K> ArrTraits;
    typedef CGAL::Arrangement_2< ArrTraits> Arrangement;
    typedef ArrTraits::Segment_2 ArrSegment;

    // typedefs for defining the adaptor
    typedef CGAL::Triangulation_vertex_base_with_info_2< LabelType, K> Vb;
    typedef CGAL::Triangulation_data_structure_2< Vb> Tds;
    typedef CGAL::Delaunay_triangulation_2< K, Tds> Delaunay;
    typedef CGAL::Delaunay_triangulation_adaptation_traits_2< Delaunay> AT;
    typedef CGAL::Delaunay_triangulation_caching_degeneracy_removal_policy_2<
        Delaunay> AP;
    typedef CGAL::Voronoi_diagram_2< Delaunay, AT, AP> Voronoi;

    typedef ::std::map< typename Voronoi::Vertex_handle,
        typename Arrangement::Vertex_handle> VertexMap;
    typedef ::std::set< typename Voronoi::Delaunay_edge> DelaunayEdgeSet;
    typedef ::std::map< const Arrangement::Halfedge *, LabelType> HalfedgeInfo;

    struct TracingData
    {
      TracingData(const Voronoi & _voronoi) :
          voronoi(_voronoi)
      {
      }

      const Voronoi & voronoi;
      VertexMap vertexMap;
      DelaunayEdgeSet toVisit;
      DelaunayEdgeSet boundaryEdges;
    };

  public:
    VoronoiEdgeTracer(const Voronoi & voronoi);

    const Arrangement &
    getArrangement() const;
    LabelType
    getInfo(const Arrangement::Halfedge_const_handle halfedge) const;

  private:
    void
    initArrangement(TracingData & tracingData);
    void
    traceEdge(TracingData & tracingData,
        typename DelaunayEdgeSet::const_iterator edgeIt);
    void
    traceEdge(TracingData & tracingData,
        const typename Voronoi::Halfedge_handle & halfEdge,
        const typename VertexMap::const_iterator source);
    bool
    isBoundaryEdge(const typename Delaunay::Edge & edge) const;
    void
    splitEdges();

    Arrangement arrangement_;
    VertexMap vertexMap_;
    HalfedgeInfo halfedgeInfo_;
  };

}
}

#include "spl/analysis/detail/VoronoiEdgeTracer.h"

#endif /* SSLIB_USE_CGAL */
#endif /* VORONOI_EDGE_TRACER_H */
