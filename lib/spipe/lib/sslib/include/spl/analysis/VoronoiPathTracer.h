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

#include "spl/analysis/MapArrangementTraits.h"
#include "spl/analysis/VoronoiPathArrangement.h"
#include "spl/utility/Range.h"

// FORWARD DECLARATIONS ///////

// DEFINITION ///////////////////////

namespace spl {
namespace analysis {

template< typename MapTraits>
  class VoronoiPathTracer
  {
    typedef typename MapTraits::Label Label;
    typedef typename MapTraits::Arrangement Map;

    typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
    typedef typename K::FT FT;

    // typedefs for defining the adaptor
    typedef CGAL::Triangulation_vertex_base_with_info_2< Label, K> Vb;
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

    typedef std::pair< Line, Point> LineAndCoM;

  public:
    typedef CGAL::Voronoi_diagram_2< Delaunay, AT, AP> Voronoi;

  public:
    Map
    generateMap(const Voronoi & voronoi) const;

  private:
    typedef std::pair< size_t, size_t> Subpath;
    typedef std::vector< size_t> PossiblePath;

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

    class DirectionChecker;
    struct PathInfo;
    struct MeetingInfo;

    static Point
    midpoint(const typename Delaunay::Edge & edge);
    Segment
    segment(const typename Delaunay::Edge & edge) const;

    PathArrangement
    generatePaths(const Voronoi & voronoi, TracingData * const tracing) const;

    std::vector< ptrdiff_t>
    findStraightPathsInternal(const Path & path,
        const PathArrangement & arr) const;
    bool
    isStraight(const Path & path, size_t vtxI, size_t vtxK) const;
    bool
    isStraight(const typename Delaunay::Edge & startEdge,
        const typename Delaunay::Edge & endEdge, const Path & path,
        const size_t pathEdgeBegin, const size_t pathEdgeLast,
        DirectionChecker * const dirs) const;

    PossiblePath
    findOptimumPath(const Path & path,
        const std::vector< ptrdiff_t> & longest) const;
    double
    penalty(const Path & fullPath, const Subpath & subpath) const;
    std::vector< Line>
    calculateLeastSquaresSubpaths(const Path & full,
        const PossiblePath & reduced, const Voronoi & voronoi) const;
    LineAndCoM
    calculateLeastSquaresLine(const Path & full, const size_t i,
        const size_t j) const;
    typename CGAL::Linear_algebraCd< K::FT>::Matrix
    calculateQuadraticForm(const LineAndCoM & line) const;

    Path
    extractReducedPath(const Path & full, const PossiblePath & reduced) const;

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

    ptrdiff_t
    pEnd(const ptrdiff_t i, const Path & p) const
    {
      return p.isClosed() ? i + p.numVertices() : p.numVertices();
    }

    ptrdiff_t
    lonWrap(const ptrdiff_t i, const Path & p) const
    {
      const ptrdiff_t n = p.numVertices();
      if(p.isClosed())
        return i < 0 ? n - (-i % n) : i % n;
      else
        return i < 0 ? n + 1 : i;
    }
  };

}
}

#include "spl/analysis/detail/VoronoiPathTracer.h"

#endif /* SPL_WITH_CGAL */
#endif /* VORONOI_PATH_TRACER_H */
