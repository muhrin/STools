/*
 * VoronoiPathTracer.h
 *
 *  Created on: Feb 18, 2014
 *      Author: Martin Uhrin
 */

#ifndef VORONOI_PATH_TRACER_DETAIL_H
#define VORONOI_PATH_TRACER_DETAIL_H

// INCLUDES ///////////////////
#include <set>
#include <iostream>

#include <boost/foreach.hpp>

#include <CGAL/linear_least_squares_fitting_2.h>
#include <CGAL/MP_Float.h>
#include <CGAL/Gmpfr.h>
#include <CGAL/Polytope_distance_d.h>
#include <CGAL/Polytope_distance_d_traits_2.h>

#include "spl/SSLibAssert.h"
#include "spl/analysis/VoronoiPathDecompose.h"
#include "spl/analysis/VoronoiPathUtility.h"
#include "spl/utility/StableComparison.h"
#include "spl/utility/TransformFunctions.h"

// FORWARD DECLARATIONS ///////

// DEFINITION ///////////////////////

namespace spl {
namespace analysis {

template< typename Voronoi>
  CGAL::Polygon_2< typename Voronoi::Delaunay_graph::Geom_traits>
  surroundingPolygon(const typename Voronoi::Vertex & vtx)
  {
    CGAL::Polygon_2< typename Voronoi::Delaunay_graph::Geom_traits> poly;
    const typename Voronoi::Halfedge_around_vertex_circulator start =
        vtx.incident_halfedges();
    typename Voronoi::Halfedge_around_vertex_circulator cl = start;
    do
    {
      poly.push_back(cl->face()->dual()->point());
      ++cl;
    } while(cl != start);
    return poly;
  }

template< typename MapTraits>
  class VoronoiPathTracer< MapTraits>::DirectionChecker
  {
    struct Direction
    {
      enum Value
      {
        UP, LEFT, DOWN, RIGHT
      };
    };
  public:
    void
    update(const Vector & dr)
    {
      namespace stable = spl::utility::stable;

      if(numDirections() == 4)
        return;

      if(stable::gt(dr.x(), 0.0))
        myDirections.insert(Direction::RIGHT);
      else if(stable::lt(dr.x(), 0.0))
        myDirections.insert(Direction::LEFT);

      if(stable::gt(dr.y(), 0.0))
        myDirections.insert(Direction::UP);
      else if(stable::lt(dr.y(), 0.0))
        myDirections.insert(Direction::DOWN);
    }
    size_t
    numDirections() const
    {
      return myDirections.size();
    }
  private:
    std::set< typename Direction::Value> myDirections;
  };

template< typename MapTraits>
  struct VoronoiPathTracer< MapTraits>::TracingData
  {
    TracingData(const Voronoi & voronoi) :
        pathArrangement(voronoi)
    {
    }
    VoronoiPathArrangement< Voronoi> pathArrangement;
  };

template< typename MapTraits>
  typename VoronoiPathTracer< MapTraits>::Map
  VoronoiPathTracer< MapTraits>::generateMap(const Voronoi & voronoi) const
  {
    TracingData tracingData(voronoi);
    const PathArrangement & pathArrangement = generatePaths(voronoi,
        &tracingData);

    return analysis::toMap< MapTraits>(pathArrangement);
  }

template< typename MapTraits>
  typename VoronoiPathTracer< MapTraits>::Point
  VoronoiPathTracer< MapTraits>::midpoint(const typename Delaunay::Edge & edge)
  {
    return CGAL::midpoint(edge.first->vertex((edge.second + 1) % 3)->point(),
        edge.first->vertex((edge.second + 2) % 3)->point());
  }

template< typename MapTraits>
  typename VoronoiPathTracer< MapTraits>::Segment
  VoronoiPathTracer< MapTraits>::segment(
      const typename Delaunay::Edge & edge) const
  {
    return Segment(edge.first->vertex((edge.second + 1) % 3)->point(),
        edge.first->vertex((edge.second + 2) % 3)->point());
  }

template< typename MapTraits>
  struct VoronoiPathTracer< MapTraits>::PathInfo
  {
    std::vector< Line> fitLines;
    const Path * orig;
    Path adjusted;
  };

template< typename MapTraits>
  typename VoronoiPathTracer< MapTraits>::PathArrangement
  VoronoiPathTracer< MapTraits>::generatePaths(const Voronoi & voronoi,
      TracingData * const tracing) const
  {
    decomposePaths(voronoi, &tracing->pathArrangement);
    //return tracing->pathArrangement;

    PathArrangement pathArr(voronoi);

    BOOST_FOREACH(const Path & path,
        boost::make_iterator_range(tracing->pathArrangement.pathsBegin(), tracing->pathArrangement.pathsEnd()))
    {
      const std::vector< ptrdiff_t> & longest = findStraightPathsInternal(path,
          tracing->pathArrangement);
      const PossiblePath & optimal = findOptimumPath(path, longest);

      Path optimalPath = extractReducedPath(path, optimal);
      adjustVertices(&optimalPath);

      pathArr.insertPath(optimalPath);
    }

    optimiseBoundaryVertices(&pathArr);
    optimiseMeetingVertices(&pathArr);

    return pathArr;
  }

template< typename MapTraits>
  std::vector< ptrdiff_t>
  VoronoiPathTracer< MapTraits>::findStraightPathsInternal(const Path & path,
      const PathArrangement & arr) const
  {
    typedef typename Path::Edge Edge;
    typedef typename PathArrangement::MeetingVerticesConst MeetingVertices;

    const ptrdiff_t n = static_cast< ptrdiff_t>(path.numVertices());

    arma::Mat< int> straight(n + 2, n + 2);
    straight.zeros();
    for(ptrdiff_t i = 0; i < n; ++i) // Start vertex
    {
      for(ptrdiff_t k = i + 1; k < pEnd(i, path); ++k) // End vertex
        straight(lonWrap(i, path), lonWrap(k, path)) = isStraight(path, i,
            path.wrapIndex(k));
    }

    if(!path.isCircular())
    {
      bool pathStraight;
      const MeetingVertices & meeting = arr.getMeetingVertices();
      std::pair< typename MeetingVertices::const_iterator,
          typename MeetingVertices::const_iterator> meetingRange;

      // Vertex before the start meeting vertex
      if(!path.vertexFront().isBoundary())
      {
        // Paths of length 1-2 are always possible
        for(ptrdiff_t i = 0; i < std::min(static_cast< ptrdiff_t>(2), n); ++i)
        {
          straight(lonWrap(-1, path), lonWrap(i, path)) = true;
        }

        meetingRange = meeting.equal_range(path.vertex(0).voronoiVertex());

        for(ptrdiff_t i = 2; i < n; ++i) // End vertex
        {
          const typename Path::Edge & e1 = path.edge(i - 1);

          pathStraight = true;
          BOOST_FOREACH(typename PathArrangement::MeetingVerticesConst::const_reference vtx,
              meetingRange)
          {
            const typename Path::Edge & e2 =
                vtx.second.second == 0 ?
                    vtx.second.first->edgeFront() :
                    vtx.second.first->edgeBack();

            DirectionChecker dirs;
            dirs.update(
                path.vertexFront().point()
                    - vtx.second.first->vertex(vtx.second.second).point());
            dirs.update(path.vertex(1).point() - path.vertexFront().point());

            if(!isStraight(e1.delaunayEdge(), e2.delaunayEdge(), path, 0, i - 2,
                &dirs))
            {
              pathStraight = false;
              break;
            }
          }
          straight(lonWrap(-1, path), lonWrap(i, path)) = pathStraight;

        }
      }

      // Vertex after the end meeting vertex
      if(n > 1 && !path.vertexBack().isBoundary())
      {
        // Paths of length 1-2 are always possible
        for(ptrdiff_t i = n - 1;
            i >= std::max(n - 2, static_cast< ptrdiff_t>(0)); --i)
        {
          straight(lonWrap(i, path), lonWrap(n, path)) = true;
        }

        meetingRange = meeting.equal_range(path.vertex(n - 1).voronoiVertex());

        for(ptrdiff_t i = n - 3; i >= 0; --i) // End vertex
        {
          const Edge & e1 = path.edge(i);

          pathStraight = true;
          BOOST_FOREACH(typename MeetingVertices::const_reference vtx,
              meetingRange)
          {
            const Edge & e2 =
                vtx.second.second == 0 ?
                    vtx.second.first->edgeFront() :
                    vtx.second.first->edgeBack();

            DirectionChecker dirs;
            dirs.update(
                path.vertex(n - 1).point() - path.vertex(n - 2).point());
            dirs.update(
                vtx.second.first->vertex(vtx.second.second).point()
                    - path.vertex(n - 1).point());

            if(!isStraight(e1.delaunayEdge(), e2.delaunayEdge(), path, i + 1,
                path.numEdges() - 1, &dirs))
            {
              pathStraight = false;
              break;
            }
          }
          straight(lonWrap(i, path), lonWrap(n, path)) = pathStraight;
        }
      }

      if(n > 1 && !path.vertexFront().isBoundary()
          && !path.vertexBack().isBoundary())
      {
        meetingRange = meeting.equal_range(path.vertexFront().voronoiVertex());
        std::pair< typename MeetingVertices::const_iterator,
            typename MeetingVertices::const_iterator> meetingRange2 =
            meeting.equal_range(path.vertexBack().voronoiVertex());
        pathStraight = true;
        BOOST_FOREACH(typename MeetingVertices::const_reference vtx,
            meetingRange)
        {
          const Edge & e1 =
              vtx.second.second == 0 ?
                  vtx.second.first->edgeFront() : vtx.second.first->edgeBack();

          BOOST_FOREACH(typename MeetingVertices::const_reference vtx2,
              meetingRange2)
          {
            const Edge & e2 =
                vtx2.second.second == 0 ?
                    vtx2.second.first->edgeFront() :
                    vtx2.second.first->edgeBack();

            DirectionChecker dirs;
            dirs.update(
                path.vertex(n - 1).point() - path.vertex(n - 2).point());
            dirs.update(
                vtx.second.first->vertex(vtx.second.second).point()
                    - path.vertex(n - 1).point());
            dirs.update(
                path.vertex(n - 1).point() - path.vertex(n - 2).point());
            dirs.update(
                vtx.second.first->vertex(vtx.second.second).point()
                    - path.vertex(n - 1).point());

            if(!isStraight(e1.delaunayEdge(), e2.delaunayEdge(), path, 0, n - 2,
                &dirs))
            {
              pathStraight = false;
              break;
            }
          }
          if(!pathStraight)
            break;
        }
        straight(lonWrap(-1, path), lonWrap(n, path)) = pathStraight;
      }
    }

    const ptrdiff_t s0 = path.isCircular() ? 0 : -1, s1 =
        path.isCircular() ? n : n + 1;
    std::vector< ptrdiff_t> longest(n + 2);
    for(ptrdiff_t i = s0; i < s1; ++i) // Start vertex
    {
      const ptrdiff_t dMax = path.isCircular() ? n : n - i + 1;
      for(ptrdiff_t d = 1; d < dMax; ++d)
      {
        const ptrdiff_t k = path.isCircular() ? path.wrapIndex(i + d) : i + d; // End vertex
        bool allStraight = true;
        for(ptrdiff_t l = i; l < k; ++l) // Start
        {
          for(ptrdiff_t m = l + 1; m <= k; ++m) // End
          {
            if(!straight(lonWrap(l, path), lonWrap(m, path)))
            {
              allStraight = false;
              break;
            }
          }
          if(!allStraight)
            break;
        }
        if(allStraight)
          longest[lonWrap(i, path)] = k;
        else
          break;
      }
    }

    if(path.vertexFront().isBoundary())
      longest[lonWrap(-1, path)] = longest[0];
    if(n > 1 && path.vertexBack().isBoundary())
    {
      BOOST_FOREACH(ptrdiff_t & i, longest)
      {
        if(i == n - 1)
          i = n;
      }
      longest[n - 1] = n;
    }

    return longest;
  }

template< typename MapTraits>
  bool
  VoronoiPathTracer< MapTraits>::isStraight(const Path & path, size_t i,
      size_t k) const
  {
    SSLIB_ASSERT(path.inRange(i));
    SSLIB_ASSERT(path.inRange(k));

    // Paths of length 0-2 are always straight
    if(path.forwardDist(i, k) <= 2)
      return true;

    // The start and end Delaunay edges
    const typename Delaunay::Edge & dgE1 = path.edge(i).delaunayEdge();
    const typename Delaunay::Edge & dgE2 =
        path.edge(path.wrapIndex(k - 1)).delaunayEdge();

    const Point v_i[] =
      { dgE1.first->vertex((dgE1.second + 1) % 3)->point(), dgE1.first->vertex(
          (dgE1.second + 2) % 3)->point() };
    const Point v_k[] =
      { dgE2.first->vertex((dgE2.second + 1) % 3)->point(), dgE2.first->vertex(
          (dgE2.second + 2) % 3)->point() };

    // Construct the 4 segments between the two endpoints of the two Delaunay edges
    Segment ik[4];
    for(size_t l = 0; l < 2; ++l)
    {
      for(size_t m = 0; m < 2; ++m)
        ik[l * 2 + m] = Segment(v_i[l], v_k[m]);
    }

    DirectionChecker dirs;
    // No need to check paths of length 1 & 2, just seed the direction vector
    dirs.update(
        path.vertex(path.wrapIndex(i + 1)).point() - path.vertex(i).point());

    bool pathStraight;
    const ptrdiff_t first = path.safeIndex(i + 2);
    for(ptrdiff_t d = 0; d < path.forwardDist(first, k); ++d) // Inbetween vertices
    {
      pathStraight = false;
      const typename Path::Edge & edge = path.edge(
          path.wrapIndex(first + d - 1));
      dirs.update(
          path.vertex(edge.target()).point()
              - path.vertex(edge.source()).point());
      // Check 4-directions condition
      if(dirs.numDirections() == 4)
        break;

      const Segment edgeSegment = segment(edge.delaunayEdge());
      for(size_t l = 0; l < 4; ++l)
      {
        if(CGAL::intersection(ik[l], edgeSegment))
        {
          // As soon as one of the segments intersects this Edge then we're
          // happy to move on
          pathStraight = true;
          break;
        }
      }
      if(!pathStraight)
        break;
    }

    // Add in the final direction vector
    dirs.update(
        path.vertex(k).point() - path.vertex(path.wrapIndex(k - 1)).point());
    if(dirs.numDirections() == 4)
      pathStraight = false;

    return pathStraight;
  }

template< typename MapTraits>
  bool
  VoronoiPathTracer< MapTraits>::isStraight(
      const typename Delaunay::Edge & startEdge,
      const typename Delaunay::Edge & endEdge, const Path & path,
      const size_t pathEdgeBegin, const size_t pathEdgeLast,
      DirectionChecker * const dirs) const
  {
    SSLIB_ASSERT(pathEdgeBegin < path.numEdges());
    SSLIB_ASSERT(pathEdgeLast < path.numEdges());
    SSLIB_ASSERT(pathEdgeBegin <= pathEdgeLast);

    const Point v_i[] =
      { startEdge.first->vertex((startEdge.second + 1) % 3)->point(),
          startEdge.first->vertex((startEdge.second + 2) % 3)->point() };
    const Point v_k[] =
      { endEdge.first->vertex((endEdge.second + 1) % 3)->point(),
          endEdge.first->vertex((endEdge.second + 2) % 3)->point() };

    // Construct the 4 segments between the two endpoints of the two Delaunay edges
    Segment ik[4];
    for(size_t l = 0; l < 2; ++l)
    {
      for(size_t m = 0; m < 2; ++m)
        ik[l * 2 + m] = Segment(v_i[l], v_k[m]);
    }

    bool pathStraight;
    for(size_t j = pathEdgeBegin; j <= pathEdgeLast; ++j) // Path edges to check
    {
      pathStraight = false;
      const typename Path::Edge & edge = path.edge(j);
      dirs->update(
          path.vertex(edge.target()).point()
              - path.vertex(edge.source()).point());
      // Check 4-directions condition
      if(dirs->numDirections() == 4)
        break;

      const Segment edgeSegment = segment(edge.delaunayEdge());
      for(size_t l = 0; l < 4; ++l)
      {
        if(CGAL::intersection(ik[l], edgeSegment))
        {
          // As soon as one of the segments intersects this Edge then we're
          // happy to move on
          pathStraight = true;
          break;
        }
      }
      if(!pathStraight)
        break;
    }

    return pathStraight;
  }

template< typename MapTraits>
  typename VoronoiPathTracer< MapTraits>::PossiblePath
  VoronoiPathTracer< MapTraits>::findOptimumPath(const Path & path,
      const std::vector< ptrdiff_t> & longest) const
  {
    const ptrdiff_t n = static_cast< ptrdiff_t>(path.numVertices());
    if(n == 0)
      return PossiblePath();
    else if(n == 1)
    {
      PossiblePath poss;
      poss.push_back(0);
      return poss;
    }

    ptrdiff_t i, j;

    // Calculate clipped paths i.e. 1 less at start and end then longest path
    size_t c;
    std::vector< ptrdiff_t> clip0(n);
    for(i = 0; i < n; ++i)
    {
      c = lonWrap(longest[lonWrap(i - 1, path)] - 1, path);
      clip0[i] = c < i ? n : c;
    }

    const ptrdiff_t endVertex = path.isCircular() ? n : n - 1;
    // Calculate seg0[j] = longest path index from 0 with j segments
    std::vector< ptrdiff_t> seg0(endVertex + 1);
    for(i = 0, j = 0; i < endVertex; ++j)
    {
      seg0[j] = i;
      i = clip0[i];
    }
    seg0[j] = endVertex;
    const ptrdiff_t m = j; // The number of segments in the full path

    // Calculate backwards clipped paths
    std::vector< ptrdiff_t> clip1(endVertex + 1);
    for(j = 1, i = 0; i < endVertex; ++i)
    {
      while(j <= clip0[i])
      {
        clip1[j] = i;
        ++j;
      }
    }

    // Calculate seg1[j] = longest path index to n with m - j segments
    std::vector< ptrdiff_t> seg1(endVertex + 1);
    for(i = endVertex, j = m; j > 0; --j)
    {
      seg1[j] = i;
      i = clip1[i];
    }
    seg1[0] = 0;

    std::vector< double> penalties(endVertex + 1);
    std::vector< ptrdiff_t> prev(endVertex + 1);
    penalties[0] = 0.0;
    for(j = 1; j <= m; j++)
    {
      for(i = seg1[j]; i <= seg0[j]; i++)
      {
        double best = -1.0;
        for(ptrdiff_t k = seg0[j - 1]; k >= clip1[i]; k--)
        {
          const double thispen = penalty(path, Subpath(k, i)) + penalties[k];
          if(best < 0 || thispen < best)
          {
            prev[i] = k;
            best = thispen;
          }
        }
        penalties[i] = best;
      }
    }

    // Get the shortest path
    PossiblePath optimal(m + 1);
    optimal.back() = path.isCircular() ? 0 : n - 1;
    for(i = endVertex, j = m - 1; i > 0; --j)
    {
      i = prev[i];
      optimal[j] = i;
    }

    return optimal;
  }


template< typename MapTraits>
  double
  VoronoiPathTracer< MapTraits>::penalty(const Path & fullPath,
      const Subpath & subpath) const
  {
    const size_t n = subpath.second - subpath.first;

    const Segment v_ij(fullPath.vertex(subpath.first).point(),
        fullPath.vertex(subpath.second).point());
    const double lenSq = CGAL::to_double(v_ij.squared_length());

    double sumSq = 0.0;
    for(size_t k = subpath.first; k < subpath.second; ++k)
      sumSq += CGAL::to_double(
          CGAL::squared_distance(fullPath.vertex(k).point(), v_ij));

    return lenSq * (1.0 / static_cast< double>(n)) * sumSq;
  }

template< typename MapTraits>
  typename VoronoiPathTracer< MapTraits>::LineAndCoM
  VoronoiPathTracer< MapTraits>::calculateLeastSquaresLine(const Path & full,
      const size_t i, const size_t j) const
  {
    SSLIB_ASSERT(full.inRange(i));
    SSLIB_ASSERT(full.inRange(j));

    std::vector< Point> pts;
    for(ptrdiff_t d = 0; d <= full.forwardDist(i, j); ++d)
      pts.push_back(full.vertex(full.wrapIndex(i + d)).point());

    LineAndCoM line;
    CGAL::linear_least_squares_fitting_2(pts.begin(), pts.end(), line.first,
        line.second, CGAL::Dimension_tag< 0>());

    return line;
  }

template< typename MapTraits>
  typename CGAL::Linear_algebraCd< typename VoronoiPathTracer< MapTraits>::K::FT>::Matrix
  VoronoiPathTracer< MapTraits>::calculateQuadraticForm(
      const LineAndCoM & line) const
  {
    typedef typename CGAL::Linear_algebraCd< K::FT> Linalg;
    // The quadratic form matrix summed for each line.  Least squares
    // distance from point (x, y) to the lines will be
    // (x, y, 1) Q (x, y, 1)^T
    typename Linalg::Matrix Q(3, 3, 0.0);

    const Vector & ortho = line.first.to_vector().perpendicular(
        CGAL::COUNTERCLOCKWISE);
    const K::FT lenSq = ortho.squared_length();
    typename Linalg::Matrix v(3, 1);
    v(0, 0) = ortho.x();
    v(1, 0) = ortho.y();
    v(2, 0) = -ortho * (line.second - CGAL::ORIGIN);
    Q += v * Linalg::transpose(v) * (1.0 / lenSq);

    return Q;
  }

template< typename MapTraits>
  std::vector< typename VoronoiPathTracer< MapTraits>::Line>
  VoronoiPathTracer< MapTraits>::calculateLeastSquaresSubpaths(
      const Path & full, const PossiblePath & reduced,
      const Voronoi & voronoi) const
  {
    SSLIB_ASSERT(!reduced.empty());
    if(reduced.size() == 1)
    {
      const typename Voronoi::Halfedge_handle he = voronoi.dual(
          full[*reduced.begin()].second);
      SSLIB_ASSERT(he->has_source() || he->has_target());

      const Point x0 =
          he->has_source() ?
              he->source()->point() : full[*reduced.begin()].first;
      const Point x1 =
          he->has_target() ?
              he->target()->point() : full[*reduced.begin()].first;

      return std::vector< Line>(1, Line(x0, x1));
    }
    else
    {
      std::vector< Line> lines(reduced.size() - 1);

      PossiblePath::const_iterator it = reduced.begin();
      const PossiblePath::const_iterator end = reduced.end();

      Subpath sp;
      sp.first = *it;
      sp.second = *++it;

      CGAL::linear_least_squares_fitting_2(full.verticesBegin() + sp.first,
          full.verticesBegin() + (sp.second + 1), lines[0],
          CGAL::Dimension_tag< 0>());

      ++it;
      for(size_t lineIdx = 1; lineIdx < lines.size(); ++lineIdx, ++it)
      {
        // Move the subpath to the next edge
        sp = Subpath(sp.second, *it);

        lines[lineIdx] = calculateLeastSquaresLine(full, sp.first, sp.second);
      }
      return lines;
    }
  }

template< typename MapTraits>
  typename VoronoiPathTracer< MapTraits>::Path
  VoronoiPathTracer< MapTraits>::extractReducedPath(const Path & full,
      const PossiblePath & reduced) const
  {
    if(reduced.empty())
      return Path();

    Path extracted(*full.getVoronoi());

    // Only iterate from the second to the second last vertex indices
    const PossiblePath::const_iterator first = reduced.begin();
    const PossiblePath::const_iterator last = --(reduced.end());

    size_t edgeIdx;

    for(PossiblePath::const_iterator it = first; it != last; ++it)
    {
      PossiblePath::const_iterator next = it;
      ++next;

      edgeIdx = extracted.push_back(full.vertex(*it), full.vertex(*next),
          full.edge(*it).delaunayEdge());
      typename Path::Edge & edge = extracted.edge(edgeIdx);

      edge.setLeastSqLine(calculateLeastSquaresLine(full, *it, *next));
      edge.setQuadraticForm(calculateQuadraticForm(*edge.getLeastSqLine()));
    }

    return extracted;
  }

template< typename MapTraits>
  void
  VoronoiPathTracer< MapTraits>::adjustVertices(Path * const path) const
  {
    if(path->numEdges() < 2)
      return;

    const typename Path::EdgeConstIterator last = --(path->edgesEnd());
    typename Path::EdgeConstIterator next;
    for(typename Path::EdgeConstIterator it = path->edgesBegin(); it != last;
        ++it)
    {
      next = it;
      ++next;

      const typename Path::Edge & e1 = *it;
      const typename Path::Edge & e2 = *next;

      if(e1.getQuadraticForm() && e2.getQuadraticForm())
      {
        typename Path::Vertex & vtx = path->vertex(e1.target());

        CGAL::Linear_algebraCd< K::FT>::Matrix Q = *e1.getQuadraticForm();
        Q += *e2.getQuadraticForm();

        vtx.point() = joinLines(Q, vtx.domain());
      }
    }
  }

template< typename MapTraits>
  void
  VoronoiPathTracer< MapTraits>::optimiseBoundaryVertices(
      PathArrangement * const arr) const
  {
    typedef typename PathArrangement::BoundaryVertices BoundaryVertices;
    typedef typename BoundaryVertices::const_iterator BoundaryVertexIterator;
    typedef std::pair< BoundaryVertexIterator, BoundaryVertexIterator> BoundaryVertexRange;

    const BoundaryVertices & boundary = arr->getBoundaryVertices();

    Polygon domain;
    bool domainSet;
    const typename Path::Edge * edge;

    for(BoundaryVertexIterator it = boundary.begin(), end = boundary.end();
        it != end; /*increment in loop body*/)
    {
      const BoundaryVertexRange range(it, boundary.upper_bound(it->first));

      CGAL::Linear_algebraCd< K::FT>::Matrix Q(3, 3, 0.0);
      domainSet = false;
      BOOST_FOREACH(typename BoundaryVertices::const_reference v, range)
      {
        const typename PathArrangement::VertexHandle & vtx = v.second;
        const Path * const path = vtx.first;

        edge = vtx.second == 0 ? &path->edgeFront() : &path->edgeBack();

        SSLIB_ASSERT(edge->getQuadraticForm());
        Q += *edge->getQuadraticForm();
        // Use the first domain we come across - they should all be
        // equivalent anyway
        if(!domainSet)
        {
          domain = path->vertex(vtx.second).domain();
          domainSet = true;
        }
      }
      SSLIB_ASSERT(domainSet);

      const Point optimal = joinLines(Q, domain);

      // Now save the optimal point to all the meeting vertices
      BOOST_FOREACH(typename BoundaryVertices::const_reference v, range)
      {
        const typename PathArrangement::VertexHandle & vtx = v.second;
        vtx.first->vertex(vtx.second).point() = optimal;
      }

      it = range.second;
    }
  }

template< typename MapTraits>
  void
  VoronoiPathTracer< MapTraits>::optimiseMeetingVertices(
      PathArrangement * const arr) const
  {
    typedef typename PathArrangement::MeetingVertices MeetingVertices;
    typedef typename MeetingVertices::const_iterator MeetingVertexIterator;
    typedef std::pair< MeetingVertexIterator, MeetingVertexIterator> MeetingVertexRange;

    const MeetingVertices & meeting = arr->getMeetingVertices();

    for(MeetingVertexIterator it = meeting.begin(), end = meeting.end();
        it != end; /*increment in loop body*/)
    {
      const MeetingVertexRange range(it, meeting.upper_bound(it->first));

      CGAL::Linear_algebraCd< K::FT>::Matrix Q(3, 3, 0.0);
      typename Path::Edge * edge;
      BOOST_FOREACH(typename MeetingVertices::const_reference v, range)
      {
        const typename PathArrangement::VertexHandle & vtx = v.second;

        edge =
            vtx.second == 0 ? &vtx.first->edgeFront() : &vtx.first->edgeBack();

        SSLIB_ASSERT(edge->getQuadraticForm());
        Q += *edge->getQuadraticForm();
      }

      const typename PathArrangement::VertexHandle & firstVtx =
          range.first->second;
      const Point optimal = joinLines(Q,
          firstVtx.first->vertex(firstVtx.second).domain());

      BOOST_FOREACH(typename MeetingVertices::const_reference v, range)
      {
        const typename PathArrangement::VertexHandle & vtx = v.second;
        vtx.first->vertex(vtx.second).point() = optimal;
      }

      it = range.second;
    }
  }

template< typename MapTraits>
  typename VoronoiPathTracer< MapTraits>::Point
  VoronoiPathTracer< MapTraits>::joinLines(
      const CGAL::Linear_algebraCd< K::FT>::Matrix & Q,
      const Polygon & boundary) const
  {
    typedef CGAL::Linear_algebraCd< K::FT> Linalg;
    typedef Linalg::Matrix AlgMatrix;
    typedef Linalg::Vector AlgVector;

    Point best = CGAL::centroid(boundary.vertices_begin(),
        boundary.vertices_end());
    K::FT minDistSq = quadDist(best, Q);

    AlgMatrix nnT(2, 2);
    for(size_t i = 0; i < 2; ++i)
    {
      for(size_t j = 0; j < 2; ++j)
        nnT(i, j) = Q(i, j);
    }

    AlgVector npTn(2);
    npTn[0] = -Q(0, 2);
    npTn[1] = -Q(1, 2);

    AlgVector x;
    K::FT D;
    if(!Linalg::linear_solver(nnT, npTn, x, D))
      return best; // TODO: Fix this, it probably happens because the lines are parallel
    const Point intersection(x[0] * (1.0 / D), x[1] * (1.0 / D));

    // Now check that it isn't outside the boundary
    if(boundary.bounded_side(intersection) != CGAL::ON_UNBOUNDED_SIDE)
      return intersection;

    K::FT distSq;
    Point bound;
    for(typename Polygon::Edge_const_iterator it = boundary.edges_begin(), end =
        boundary.edges_end(); it != end; ++it)
    {
      const Line boundaryLine = it->supporting_line();

      // Parametric parameters
      const Point p0 = it->source();

      const Vector lineVec = it->to_vector();
      const Vector ortho = lineVec.perpendicular(CGAL::COUNTERCLOCKWISE);

      // Project a line orthogonal to the edge toward the intersection point
      // (i.e. the closest approach) but limit it to be on the boundary
      K::FT t = ortho * (intersection - CGAL::ORIGIN);
      t = CGAL::min(CGAL::max(t, FT(0.0)), FT(1.0));
      bound = p0 + t * lineVec;

      distSq = quadDist(bound, Q);
      if(distSq < minDistSq)
      {
        minDistSq = distSq;
        best = bound;
      }
    }

    return best;
  }

template< typename MapTraits>
  typename VoronoiPathTracer< MapTraits>::Polygon
  VoronoiPathTracer< MapTraits>::surroundingBoundary(
      const typename Delaunay::Edge & edge, const Voronoi & voronoi) const
  {
    const typename Voronoi::Halfedge_handle he = voronoi.dual(edge);
    SSLIB_ASSERT(!he->is_unbounded());

    const typename Voronoi::Vertex_handle source = he->source();
    const typename Voronoi::Vertex_handle target = he->target();

    Polygon poly;
    typename Voronoi::Halfedge_around_vertex_circulator start =
        voronoi.incident_halfedges(target, he);
    typename Voronoi::Halfedge_around_vertex_circulator cl = start;
    do
    {
      poly.push_back(cl->face()->dual()->point());
      ++cl;
    } while(cl != start);

    cl = voronoi.incident_halfedges(source, he->twin());
    typename Voronoi::Halfedge_around_vertex_circulator end = cl;
    --end;
    ++cl;
    do
    {
      poly.push_back(cl->face()->dual()->point());
      ++cl;
    } while(cl != end);

    return poly;
  }

template< typename MapTraits>
  typename VoronoiPathTracer< MapTraits>::K::FT
  VoronoiPathTracer< MapTraits>::quadDist(const Point & p,
      const CGAL::Linear_algebraCd< K::FT>::Matrix & Q) const
  {
    typename CGAL::Linear_algebraCd< K::FT>::Matrix v(3, 1);
    v(0, 0) = p.x();
    v(1, 0) = p.y();
    v(2, 0) = 1;

    return (CGAL::Linear_algebraCd< K::FT>::transpose(v) * Q * v)(0, 0);
  }

}
}

#endif /* VORONOI_PATH_TRACER_DETAIL_H */
