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

#include "spl/analysis/MapArrangement.h"
#include "spl/analysis/VoronoiPathArrangement.h"
#include "spl/utility/Range.h"

// FORWARD DECLARATIONS ///////

// DEFINITION ///////////////////////

namespace spl {
namespace analysis {

template< typename LabelType>
  class VoronoiPathTracer
  {
    typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
    typedef typename K::FT FT;

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

    //typedef typename MapArrangement< K, LabelType>::Arrangement Map;
    typedef typename MapArrangement< CGAL::Exact_predicates_exact_constructions_kernel, LabelType>::Arrangement Map;

  public:
    Map
    generateMap(const Voronoi & voronoi) const;

  private:
    typedef boost::adjacency_list< > PathGraph;
    typedef std::map< PathGraph::vertex_descriptor,
        std::vector< PathGraph::vertex_descriptor> > IncomingMap;
    typedef std::pair< size_t, size_t> Subpath;
    typedef std::set< size_t> PossiblePath;
    typedef std::map< PathGraph::edge_descriptor, double> WeightMap;

    struct PathGraphInfo
    {
      PathGraph graph;
      std::map< ptrdiff_t, typename PathGraph::vertex_descriptor> vertexMap;
    };

    struct NextHalfedgeType
    {
      enum Value
      {
        IS_BOUNDARY, IS_NULL, IS_START
      };
    };
  public:
    typedef VoronoiPath< Voronoi> Path;
    typedef VoronoiPathArrangement< Voronoi> PathArrangement;
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

    PathArrangement
    generatePaths(const Voronoi & voronoi, TracingData * const tracing) const;

    PathGraphInfo
    findStraightPathsInternal(const Path & path,
        const PathArrangement & arr) const;
    PathGraphInfo
    findStraightPathsBoundary(const Path & path,
        const TracingData & tracing) const;

    void
    filterPossiblePaths(const Path & path, PathGraphInfo * const paths) const;
    PossiblePath
    findOptimumPath(const Path & path, const PathGraphInfo & paths) const;
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
    extractReducedPath(const Path & full, const PossiblePath & reduced) const;

    void
    placeMeetingVertices(TracingData * const tracing) const;

    void
    adjustVertices(Path * const path) const;
    void
    optimiseBoundaryVertices(PathArrangement * const arr) const;
    void
    optimiseMeetingVertices(PathArrangement * const arr) const;

    Point
    joinLines(const CGAL::Linear_algebraCd< K::FT>::Matrix & quad,
        const Polygon & boundary) const;
    Polygon
    surroundingBoundary(const typename Delaunay::Edge & edge,
        const Voronoi & voronoi) const;
    K::FT
    quadDist(const Point & p,
        const CGAL::Linear_algebraCd< K::FT>::Matrix & Q) const;
  };

}
}

#include "spl/analysis/detail/VoronoiPathTracer.h"

#endif /* SPL_WITH_CGAL */
#endif /* VORONOI_PATH_TRACER_H */
