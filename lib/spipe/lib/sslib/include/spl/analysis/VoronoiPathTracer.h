/*
 * VoronoiPathTracer.h
 *
 *  Created on: Feb 18, 2014
 *      Author: Martin Uhrin
 */

#ifndef VORONOI_PATH_TRACER_H
#define VORONOI_PATH_TRACER_H

// INCLUDES ////////////
#include "spl/SSLib.h"

#ifdef SPL_WITH_CGAL

#include <map>
#include <set>

#include <boost/variant.hpp>
#include <boost/graph/adjacency_list.hpp>

#include <CGAL/Arrangement_2.h>
#include <CGAL/Arr_extended_dcel.h>
#include <CGAL/Arr_segment_traits_2.h>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Voronoi_diagram_2.h>
#include <CGAL/Delaunay_triangulation_adaptation_traits_2.h>
#include <CGAL/Delaunay_triangulation_adaptation_policies_2.h>
#include <CGAL/Linear_algebraCd.h>
#include <CGAL/Triangulation_vertex_base_with_info_2.h>
#include <CGAL/Voronoi_diagram_2.h>

#include "spl/utility/Range.h"

// FORWARD DECLARATIONS ///////

// DEFINITION ///////////////////////

namespace spl {
namespace analysis {
namespace detail {

template< typename VD>
  struct TracingArrangement
  {
    typedef VD Voronoi;
    typedef typename Voronoi::Delaunay_graph Delaunay;
    typedef typename Delaunay::Geom_traits K;
    typedef CGAL::Linear_algebraCd< typename K::FT> Linalg;

    struct FeatureLocation
    {
      enum Value { INTERIOR, BOUNDARY };
    };

    struct VertexInfo
    {
      typename Delaunay::Edge spanningEdge;
      FeatureLocation::Value location;
    };

    struct HalfedgeInfo
    {
      boost::optional< typename K::Line_2> leastSqLine;
      boost::optional< typename Linalg::Matrix> quadraticForm;
      FeatureLocation::Value location;
    };

    struct FaceInfo
    {
    };

    // Arrangements stuff
    typedef CGAL::Arr_segment_traits_2< K> ArrTraits;
    typedef CGAL::Arr_extended_dcel< ArrTraits, VertexInfo, HalfedgeInfo,
        FaceInfo> Dcel;
    typedef CGAL::Arrangement_2< ArrTraits, Dcel> Arrangement;

    typedef std::pair< typename Arrangement::Vertex_handle,
        typename Arrangement::Vertex_handle> Path;

    static const HalfedgeInfo &
    edgeInfo(const typename Arrangement::Halfedge & he)
    {
      if(he.data().leastSqLine)
        return he.data();
      else
        return he.twin()->data();
    }
  };
}

template< typename LabelType>
  class VoronoiPathTracer
  {
    typedef CGAL::Exact_predicates_inexact_constructions_kernel K;

    // typedefs for defining the adaptor
    typedef CGAL::Triangulation_vertex_base_with_info_2< LabelType, K> Vb;
    typedef CGAL::Triangulation_data_structure_2< Vb> Tds;

  public:
    typedef K::Line_2 Line;
    typedef K::Point_2 Point;
    typedef CGAL::Polygon_2< K> Polygon;
    typedef K::Segment_2 Segment;
    typedef K::Vector_2 Vector;
    typedef CGAL::Delaunay_triangulation_2< K, Tds> Delaunay;

  private:
    typedef CGAL::Delaunay_triangulation_adaptation_traits_2< Delaunay> AT;
    typedef CGAL::Delaunay_triangulation_caching_degeneracy_removal_policy_2<
        Delaunay> AP;

  public:
    typedef CGAL::Voronoi_diagram_2< Delaunay, AT, AP> Voronoi;

    struct VertexInfo
    {
    };

    struct HalfedgeInfo
    {
      HalfedgeInfo() :
          label()
      {
      }
      LabelType label;
    };

    struct FaceInfo
    {
      LabelType label;
    };

    // Arrangements stuff
    typedef CGAL::Arr_segment_traits_2< K> ArrTraits;
    typedef CGAL::Arr_extended_dcel< ArrTraits, VertexInfo, HalfedgeInfo,
        FaceInfo> Dcel;
    typedef CGAL::Arrangement_2< ArrTraits, Dcel> Arrangement;

  public:
    Arrangement
    tracePaths(const Voronoi & voronoi) const;

  private:
    typedef detail::TracingArrangement< Voronoi> TracingArrangement;

    typedef spl::utility::OrderedPair< LabelType> BoundaryPair;
    typedef boost::adjacency_list< > PathGraph;
    typedef std::map< PathGraph::vertex_descriptor,
        std::vector< PathGraph::vertex_descriptor> > IncomingMap;
    typedef std::pair< size_t, size_t> Subpath;
    typedef std::set< size_t> PossiblePath;
    typedef std::map< PathGraph::edge_descriptor, double> WeightMap;

    struct NextHalfedgeType
    {
      enum Value
      {
        IS_BOUNDARY, IS_NULL, IS_START
      };
    };
  public:
    class Path;
  private:
    struct TracingData;

    template< typename Func>
      class BoundaryTraceVisitor;
    class GeneratePathVisitor;
    struct PathInfo;
    struct MeetingInfo;

    static Point
    midpoint(const typename Delaunay::Edge & edge);
    Segment
    segment(const typename Delaunay::Edge & edge) const;

    void
    generatePaths(const Voronoi & voronoi, TracingData * const tracing) const;
    void
    generatePathVertices(const Voronoi & voronoi,
        TracingData * const tracing) const;
    void
    smoothPaths(const Voronoi & voronoi, TracingData * const tracing,
        Arrangement * const arr) const;
    bool
    isBoundary(const typename Voronoi::Halfedge & he) const;
    bool
    spansBoundary(const typename Delaunay::Edge & edge) const;
    BoundaryPair
    getBoundaryPair(const typename Voronoi::Halfedge & he) const;
    BoundaryPair
    getBoundaryPair(const typename Delaunay::Edge & edge) const;

    typename Voronoi::Halfedge_handle
    targetBoundaryHalfedge(const typename Voronoi::Halfedge_handle & he) const;
    template< typename Visitor>
      void
      visitBoundaryHaledges(const Voronoi & voronoi,
          const typename Voronoi::Halfedge_handle & start,
          Visitor visitor) const;

    PathGraph
    findStraightPaths(const Path & path, const TracingData & tracing) const;
    void
    filterPossiblePaths(const Path & path, PathGraph * const paths) const;
    PossiblePath
    findOptimumPath(const Path & path, const PathGraph & paths) const;
    void
    generatePossiblePaths(const Path & fullPath, const IncomingMap & incoming,
        std::vector< PossiblePath> * const possiblePaths) const;
    void
    generatePossiblePaths(const IncomingMap & incoming,
        const PathGraph::vertex_descriptor currentVertex,
        PossiblePath * const currentPath,
        std::vector< PossiblePath> * const possiblePaths) const;
    WeightMap
    calculatePenalties(const Path & fullPath,
        const PathGraph & shortestPaths) const;
    double
    penalty(const Path & fullPath, const Subpath & subpath) const;
    std::vector< Line>
    calculateLeastSquaresSubpaths(const Path & full,
        const PossiblePath & reduced, const Voronoi & voronoi) const;
    Line
    calculateLeastSquaresLine(const Path & full, const size_t idx0,
        const size_t idx1) const;
    typename CGAL::Linear_algebraCd< K::FT>::Matrix
    calculateQuadraticForm(const Line & line) const;
    Path
    generateVertexAdjustedPath(const Path & path, const PossiblePath & reduced,
        const Voronoi & voronoi, const std::vector< Line> & fitLines) const;
    typename TracingArrangement::Path
    pathToArrangement(const Path & path, const PossiblePath & reduced,
        typename TracingArrangement::Arrangement * const arr) const;
    void
    vertexAdjustment(const Voronoi & voronoi,
        typename TracingArrangement::Arrangement * const arr) const;
    template< typename LineIterator>
      Point
      joinLines(LineIterator begin, LineIterator beyond,
          const Polygon & boundary) const;
    Point
    joinLines(const CGAL::Linear_algebraCd< K::FT>::Matrix & quad,
        const Polygon & boundary) const;
    Polygon
    surroundingBoundary(const typename Delaunay::Edge & edge,
        const Voronoi & voronoi) const;
    K::FT
    quadDist(const Point & p,
        const CGAL::Linear_algebraCd< K::FT>::Matrix & Q) const;
    Point
    constrainOnSegment(const Point & p, const Segment & seg) const;
  };

}
}

#include "spl/analysis/detail/VoronoiPathTracer.h"

#endif /* SPL_WITH_CGAL */
#endif /* VORONOI_PATH_TRACER_H */
