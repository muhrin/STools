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
#include <boost/graph/breadth_first_search.hpp>

#include <CGAL/linear_least_squares_fitting_2.h>
#include <CGAL/MP_Float.h>
#include <CGAL/Gmpfr.h>
#include <CGAL/Polytope_distance_d.h>
#include <CGAL/Polytope_distance_d_traits_2.h>

#include "spl/SSLibAssert.h"

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

template< typename LabelType>
  std::ostream &
  operator <<(std::ostream & os,
      const typename VoronoiPathTracer< LabelType>::Path & path)
  {
    BOOST_FOREACH(const typename VoronoiPathTracer< LabelType>::Point & pt, path.vertices)
      os << pt << "\n";
    os << "\n" << std::endl;
    return os;
  }

template< typename IncomingMap, typename Vertex>
  class RecordIncoming : public boost::bfs_visitor< >
  {
  public:
    RecordIncoming(IncomingMap incoming, Vertex finishVertex) :
        myIncoming(incoming), myFinishVertex(finishVertex)
    {
    }

    template< class Edge, class Graph>
      void
      tree_edge(Edge e, Graph & g)
      {
        // set the parent of the target(e) to source(e)
        // TODO: Don't use push_back so other container type can be used
        myIncoming[boost::target(e, g)].push_back(boost::source(e, g));
      }

    template< class Graph>
      void
      finish_vertex(Vertex u, Graph & g)
      {
        if(u == myFinishVertex)
          throw 0;
      }

  private:
    IncomingMap myIncoming;
    Vertex myFinishVertex;
  };

template< typename LabelType>
  void
  VoronoiPathTracer< LabelType>::Path::insert(const Point & vtx,
      const typename Delaunay::Edge & spanningEdge)
  {
    vertices.push_back(vtx);
    spanningEdges.push_back(spanningEdge);
  }

template< typename LabelType>
  typename VoronoiPathTracer< LabelType>::Path
  VoronoiPathTracer< LabelType>::Path::extract(
      const PossiblePath & possible) const
  {
    Path extracted;
    BOOST_FOREACH(const size_t & idx, possible)
      extracted.insert(vertices[idx], spanningEdges[idx]);
    return extracted;
  }

template< typename LabelType>
  class VoronoiPathTracer< LabelType>::GeneratePathVisitor : public std::binary_function<
      typename Voronoi::Halfedge_handle, const typename NextHalfedgeType::Value,
      void>
  {
  public:
    GeneratePathVisitor(Path * const path,
        MeetingVertices * const meetingVertices);
    void
    operator()(typename Voronoi::Halfedge_handle he,
        const typename NextHalfedgeType::Value nextType);
  private:
    bool
    isContinuation(const typename Delaunay::Edge & edge) const;

    Path * const myPath;
    MeetingVertices * const myMeetingVertices;
  };

template< typename LabelType>
  VoronoiPathTracer< LabelType>::GeneratePathVisitor::GeneratePathVisitor(
      Path * const path, MeetingVertices * const meetingVertices) :
      myPath(path), myMeetingVertices(meetingVertices)
  {
    SSLIB_ASSERT(myPath);
  }

template< typename LabelType>
  void
  VoronoiPathTracer< LabelType>::GeneratePathVisitor::operator()(
      typename Voronoi::Halfedge_handle he,
      const typename NextHalfedgeType::Value nextType)
  {
    // Place a new vertex at the midpoint of the Delaunay edge
    // dual to this Voronoi halfedge
    const typename Delaunay::Edge & dual = he->dual();
    // Check if this is a continuation of the path i.e. this halfedges is not the last
    // one that we inserted into the path
    if(!isContinuation(dual))
      myPath->insert(midpoint(dual), dual);

    if(nextType == NextHalfedgeType::IS_START)
      myPath->isCircular = true;
    else if(nextType == NextHalfedgeType::IS_NULL)
    {
      // If we're at the end of the path it's either at the boundary of the data set
      // in which case the halfedge does not have a target (and we should do nothing)
      // or it's at a meeting vertex with another path in which case we should add
      // the vertex
      if(he->has_target())
      {
        const typename Voronoi::Vertex_handle target = he->target();
        const Polygon poly = surroundingPolygon< Voronoi>(*target);
        const Point meetingPoint =
            myMeetingVertices->insert(
                std::make_pair(target,
                    CGAL::centroid(poly.vertices_begin(), poly.vertices_end()))).first->second;
      }
    }
  }

template< typename LabelType>
  bool
  VoronoiPathTracer< LabelType>::GeneratePathVisitor::isContinuation(
      const typename Delaunay::Edge & edge) const
  {
    return !myPath->spanningEdges.empty()
        && myPath->spanningEdges.back() == edge;
  }

template< typename LabelType>
  typename VoronoiPathTracer< LabelType>::Arrangement
  VoronoiPathTracer< LabelType>::tracePaths(const Voronoi & voronoi) const
  {
    TracingData tracingData;
    Arrangement arr;
    generatePaths(voronoi, &tracingData, &arr);
    smoothPaths(voronoi, &tracingData, &arr);

    return arr;
  }

template< typename LabelType>
  void
  VoronoiPathTracer< LabelType>::Path::reverse()
  {
    std::reverse(vertices.begin(), vertices.end());
    std::reverse(spanningEdges.begin(), spanningEdges.end());
  }

template< typename LabelType>
  typename VoronoiPathTracer< LabelType>::Point
  VoronoiPathTracer< LabelType>::midpoint(const typename Delaunay::Edge & edge)
  {
    return CGAL::midpoint(edge.first->vertex((edge.second + 1) % 3)->point(),
        edge.first->vertex((edge.second + 2) % 3)->point());
  }

template< typename LabelType>
  typename VoronoiPathTracer< LabelType>::Segment
  VoronoiPathTracer< LabelType>::segment(
      const typename Delaunay::Edge & edge) const
  {
    return Segment(edge.first->vertex((edge.second + 1) % 3)->point(),
        edge.first->vertex((edge.second + 2) % 3)->point());
  }

template< typename LabelType>
  void
  VoronoiPathTracer< LabelType>::generatePaths(const Voronoi & voronoi,
      TracingData * const tracing, Arrangement * const arr) const
  {
    generatePathVertices(voronoi, tracing, arr);
    BOOST_FOREACH(const Path & path, tracing->paths)
    {
      PathGraph paths = findStraightPaths(path, *tracing);
      operator <<< LabelType>(std::cout, path);
      const PossiblePath & optimal = findOptimumPath(path, paths);
      const Path & adjusted = generateVertexAdjustedPath(path, optimal,
          voronoi);
      operator <<< LabelType>(std::cout, path.extract(optimal));
      operator <<< LabelType>(std::cout, adjusted);
    }
  }

template< typename LabelType>
  void
  VoronoiPathTracer< LabelType>::generatePathVertices(const Voronoi & voronoi,
      TracingData * const tracing, Arrangement * const arr) const
  {
    // Fist we need to place all the vertices we're going to use into the
    // unbounded face of the arrangement as this make the subsequent step
    // of creating the edges a lot easier

    const typename Voronoi::Delaunay_graph & delaunay = voronoi.dual();

    std::set< typename Delaunay::Edge> visited;
    typename Voronoi::Halfedge_handle he;
    typename Delaunay::Edge dual;
    for(typename Voronoi::Edge_iterator it = voronoi.edges_begin(), end =
        voronoi.edges_end(); it != end; ++it)
    {
      dual = it->dual();
      if(spansBoundary(dual) && visited.find(dual) == visited.end()
          && visited.find(delaunay.mirror_edge(dual)) == visited.end())
      {
        Path path;
        he = *it;
        // Trace the path out in one direction
        visitBoundaryHaledges(voronoi, he,
            GeneratePathVisitor(&path, &tracing->meetingVertices));
        if(!path.isCircular)
        {
          // Reverse the path and get the visitor to continue in the other direction
          path.reverse();
          he = he->twin();
          visitBoundaryHaledges(voronoi, he,
              GeneratePathVisitor(&path, &tracing->meetingVertices));
        }
        visited.insert(path.spanningEdges.begin(), path.spanningEdges.end());
        tracing->paths.push_back(path);
      }
    }
  }

template< typename LabelType>
  void
  VoronoiPathTracer< LabelType>::smoothPaths(const Voronoi & voronoi,
      TracingData * const tracing, Arrangement * const arr) const
  {

  }

template< typename LabelType>
  bool
  VoronoiPathTracer< LabelType>::isBoundary(
      const typename Voronoi::Halfedge & he) const
  {
    return spansBoundary(he.dual());
  }

template< typename LabelType>
  bool
  VoronoiPathTracer< LabelType>::spansBoundary(
      const typename Delaunay::Edge & edge) const
  {
    return edge.first->vertex((edge.second + 1) % 3)->info()
        != edge.first->vertex((edge.second + 2) % 3)->info();
  }

template< typename LabelType>
  typename VoronoiPathTracer< LabelType>::BoundaryPair
  VoronoiPathTracer< LabelType>::getBoundaryPair(
      const typename Voronoi::Halfedge & he) const
  {
    return getBoundaryPair(he.dual());
  }

template< typename LabelType>
  typename VoronoiPathTracer< LabelType>::BoundaryPair
  VoronoiPathTracer< LabelType>::getBoundaryPair(
      const typename Delaunay::Edge & edge) const
  {
    return BoundaryPair(edge.first->vertex((edge.second + 1) % 3)->info(),
        edge.first->vertex((edge.second + 2) % 3)->info());
  }

template< typename LabelType>
  typename VoronoiPathTracer< LabelType>::Voronoi::Halfedge_handle
  VoronoiPathTracer< LabelType>::targetBoundaryHalfedge(
      const typename Voronoi::Halfedge_handle & he) const
  {
    if(!he->has_target())
      return typename Voronoi::Halfedge_handle();

    const BoundaryPair & boundaryPair = getBoundaryPair(*he);

    typename Voronoi::Halfedge_handle next;
    const typename Voronoi::Vertex_handle target = he->target();
    const typename Voronoi::Halfedge_around_vertex_circulator start =
        target->incident_halfedges();
    typename Voronoi::Halfedge_around_vertex_circulator cl = start;
    do
    {
      if(typename Voronoi::Halfedge_handle(cl) != he
          && boundaryPair == getBoundaryPair(*cl))
      {
        next = cl->twin();
        break;
      }
      ++cl;
    } while(cl != start);
    return next;
  }

template< typename LabelType>
  template< typename Visitor>
    void
    VoronoiPathTracer< LabelType>::visitBoundaryHaledges(
        const Voronoi & voronoi,
        const typename Voronoi::Halfedge_handle & start, Visitor visitor) const
    {
      SSLIB_ASSERT(isBoundary(*start));

      const BoundaryPair boundaryPair = getBoundaryPair(*start);

      typename Voronoi::Halfedge_handle next, he = start;
      typename NextHalfedgeType::Value nextType;
      do
      {
        next = targetBoundaryHalfedge(he);
        if(next == typename Voronoi::Halfedge_handle())
          nextType = NextHalfedgeType::IS_NULL;
        else if(next == start)
          nextType = NextHalfedgeType::IS_START;
        else
          nextType = NextHalfedgeType::IS_BOUNDARY;

        visitor(he, nextType);

        he = next;
      } while(!(nextType == NextHalfedgeType::IS_NULL
          || nextType == NextHalfedgeType::IS_START));
    }

template< typename LabelType>
  typename VoronoiPathTracer< LabelType>::Path
  VoronoiPathTracer< LabelType>::joinPaths(const Path & p1,
      const Path & p2) const
  {
    // Can't join circular paths
    SSLIB_ASSERT(!p1.isCircular);
    SSLIB_ASSERT(!p2.isCircular);

    if(p1.vertices.empty() || p2.vertices.empty())
    {
      if(!p1.vertices.empty())
        return p1;
      else if(!p2.vertices.empty())
        return p2;
      else
        Path();
    }

    SSLIB_ASSERT(p1.vertices.front() == p2.vertices.front());

    Path joined;
    std::copy(p1.vertices.rbegin(), p1.vertices.rend(),
        std::back_inserter(joined.vertices));
    std::copy(p2.vertices.begin() + 1, p2.vertices.end(),
        std::back_inserter(joined.vertices));
    return joined;
  }

template< typename LabelType>
  typename VoronoiPathTracer< LabelType>::PathGraph
  VoronoiPathTracer< LabelType>::findStraightPaths(const Path & path,
      const TracingData & tracing) const
  {
    // TODO: Add check for all four directions

    const size_t n = path.vertices.size();
    PathGraph paths;
    bool pathStraight;
    for(size_t i = 0; i < n; ++i) // Start vertex
    {
      const Point & v_i = path.vertices[i];

      // Paths of length 1 are always possible
      for(size_t k = i + 1; k < std::min(i + 2, n); ++k)
        boost::add_edge(i, i + 1, paths);

      // Check longer paths
      for(size_t k = i + 2; k < n; ++k) // End vertex
      {
        const Point & v_k = path.vertices[k];
        const Segment ik(v_i, v_k);
        pathStraight = true;
        for(size_t j = i + 1; j < k; ++j) // Inbetween vertices
        {
          const Segment edgeSegment = segment(path.spanningEdges[j]);
          if(!CGAL::intersection(ik, edgeSegment))
          {
            pathStraight = false;
            break;
          }
        }
        if(pathStraight)
          boost::add_edge(i, k, paths);
      }
    }
    //filterPossiblePaths(path, &paths);
    return paths;
  }

template< typename LabelType>
  void
  VoronoiPathTracer< LabelType>::filterPossiblePaths(const Path & path,
      PathGraph * const paths) const
  {
    typedef std::pair< PathGraph::edge_descriptor, bool> EdgeQuery;

    const size_t n = path.vertices.size();
    if(n < 4)
      return;

    EdgeQuery edge;
    if(path.isCircular)
    {
      // TODO: Implement, and possibly make one piece of code with the non circular case
      std::cout << "IS CIRCULAR\n";
    }
    else
    {
      std::vector< PathGraph::edge_descriptor> toRemove;
      size_t iBack, jForward;
      for(size_t i = 0; i < n; ++i)
      {
        for(size_t j = i + 3; j < n; ++j)
        {
          edge = boost::edge(i, j, *paths);
          if(edge.second)
          {
            iBack = i > 0 ? i - 1 : i;
            jForward = j < n - 1 ? j + 1 : j;
            if(!boost::edge(iBack, jForward, *paths).second)
              toRemove.push_back(edge.first);
          }
        }
      }
      BOOST_FOREACH(const PathGraph::edge_descriptor & e, toRemove)
        paths->remove_edge(e);
    }
  }

template< typename LabelType>
  typename VoronoiPathTracer< LabelType>::PossiblePath
  VoronoiPathTracer< LabelType>::findOptimumPath(const Path & path,
      const PathGraph & paths) const
  {
    const size_t n = path.vertices.size();
    if(n == 1)
      return PossiblePath();

    const PathGraph::vertex_descriptor source = 0;
    const PathGraph::vertex_descriptor target = n - 1;
    IncomingMap incoming;
    // Make the source it's own root to allow general algorithms
    // i.e. not have to conditionally check for source and do special case
    incoming[source].push_back(source);

    RecordIncoming< IncomingMap &, PathGraph::vertex_descriptor> visitor(
        incoming, target);
    try
    {
      boost::breadth_first_search(paths, source, boost::visitor(visitor));
    }
    catch(const int)
    {
    }

    std::vector< PossiblePath> possiblePaths;
    generatePossiblePaths(path, incoming, &possiblePaths);

    PossiblePath optimal;
    if(possiblePaths.size() == 1)
      optimal = possiblePaths.front();
    else
    {
      PathGraph shortestPaths(n);
      BOOST_FOREACH(const PossiblePath & possible, possiblePaths)
      {
        size_t lastVtx = 0;
        PossiblePath::const_iterator it = possible.begin();
        ++it;
        const PossiblePath::const_iterator end = possible.end();
        for(; it != end; ++it)
        {
          boost::add_edge(lastVtx, *it, shortestPaths);
          lastVtx = *it;
        }
      }
      const WeightMap weights = calculatePenalties(path, shortestPaths);

      std::cout << "DEBUG: Found multiple possible paths of same length.\n";

      // TODO: Find the shortest path through the weighted graph using Dijkstra's algorithm
      optimal = possiblePaths.front(); // TEMP
    }

    return optimal;
  }

template< typename LabelType>
  void
  VoronoiPathTracer< LabelType>::generatePossiblePaths(const Path & fullPath,
      const IncomingMap & incoming,
      std::vector< PossiblePath> * const possiblePaths) const
  {
    if(incoming.empty())
      return;
    PossiblePath & currentPath = *possiblePaths->insert(possiblePaths->end(),
        PossiblePath());
    const size_t finalVertex = fullPath.vertices.size() - 1;
    currentPath.insert(finalVertex);
    generatePossiblePaths(incoming, finalVertex, &currentPath, possiblePaths);
  }

template< typename LabelType>
  void
  VoronoiPathTracer< LabelType>::generatePossiblePaths(
      const IncomingMap & incoming,
      const PathGraph::vertex_descriptor currentVertex,
      PossiblePath * const currentPath,
      std::vector< PossiblePath> * const possiblePaths) const
  {
    if(currentVertex == 0)
      return;

    const IncomingMap::const_iterator it = incoming.find(currentVertex);
    SSLIB_ASSERT(it != incoming.end());
    SSLIB_ASSERT(!it->second.empty());

    // Make a copy of the path so far
    const PossiblePath pathSoFar = *currentPath;

    // Continue the current path
    const size_t predecessor = it->second[0];
    currentPath->insert(predecessor);
    generatePossiblePaths(incoming, predecessor, currentPath, possiblePaths);

    // Do any branching paths
    for(size_t i = 1; i < it->second.size(); ++i)
    {
      const size_t predecessor = it->second[i];
      PossiblePath & splitPath = *possiblePaths->insert(possiblePaths->end(),
          pathSoFar);
      splitPath.insert(predecessor);
      generatePossiblePaths(incoming, predecessor, &splitPath, possiblePaths);
    }
  }

template< typename LabelType>
  typename VoronoiPathTracer< LabelType>::WeightMap
  VoronoiPathTracer< LabelType>::calculatePenalties(const Path & fullPath,
      const PathGraph & shortestPaths) const
  {
    WeightMap weights;

    BOOST_FOREACH(const typename PathGraph::edge_descriptor & edge,
        boost::edges(shortestPaths))
    {
      weights[edge] = penalty(fullPath,
          Subpath(boost::source(edge, shortestPaths),
              boost::target(edge, shortestPaths)));
    }

    return weights;
  }

template< typename LabelType>
  double
  VoronoiPathTracer< LabelType>::penalty(const Path & fullPath,
      const Subpath & subpath) const
  {
    const size_t n = subpath.second - subpath.first;

    const Segment v_ij(fullPath.vertices[subpath.first],
        fullPath.vertices[subpath.second]);
    const double lenSq = CGAL::to_double(v_ij.squared_length());

    double sumSq = 0.0;
    for(size_t k = subpath.first; k < subpath.second; ++k)
    {
      sumSq += CGAL::to_double(
          CGAL::squared_distance(fullPath.vertices[k], v_ij));
    }
    return lenSq * 1 / static_cast< double>(n) * sumSq;
  }

template< typename LabelType>
  typename VoronoiPathTracer< LabelType>::Path
  VoronoiPathTracer< LabelType>::generateVertexAdjustedPath(const Path & path,
      const PossiblePath & reduced, const Voronoi & voronoi) const
  {
    if(path.vertices.size() == 1)
      return path;

    // 1) get least squares fit through all points in subpath
    // 2) Check that the line goes through the Spanning edges at

    Path adjusted;

    adjusted.insert(path.vertices.front(), path.spanningEdges.front());

    PossiblePath::const_iterator it = reduced.begin();
    const PossiblePath::const_iterator end = reduced.end();
    Subpath sp1, sp2;
    Line l1, l2;
    sp1.first = *it;
    sp1.second = *++it;

    CGAL::linear_least_squares_fitting_2(path.vertices.begin(),
        path.vertices.begin() + (sp1.second + 1), l1,
        CGAL::Dimension_tag< 0>());

    // TODO: Intersect l1 and line supporting spanningEdges[0] to determine start point

    ++it;
    for(; it != end; ++it, l1 = l2, sp1 = sp2)
    {
      sp2 = Subpath(sp1.second, *it);

      CGAL::linear_least_squares_fitting_2(path.vertices.begin() + sp2.first,
          path.vertices.begin() + (sp2.second + 1), l2,
          CGAL::Dimension_tag< 0>());

      const typename CGAL::cpp11::result_of< K::Intersect_2
      (Line, Line)>::type result = CGAL::intersection(l1, l2);
      const Point * const intersection = boost::get< Point>(&*result);
      SSLIB_ASSERT(intersection);

      //const Point constrained = *intersection;
      const Point constrained = constrainVertex(*intersection, voronoi,
          *(path.spanningEdges.begin() + sp2.first));

      adjusted.insert(constrained, path.spanningEdges[sp1.first]);
    }

    // TODO: Intersect l1 and line supporting spanningEdges[n - 1] to determine end point
    adjusted.insert(path.vertices.back(), path.spanningEdges.back());

    return adjusted;
  }

template< typename LabelType>
  typename VoronoiPathTracer< LabelType>::Point
  VoronoiPathTracer< LabelType>::constrainVertex(const Point & point,
      const Voronoi & voronoi, const typename Delaunay::Edge & edge) const
  {
    typedef CGAL::MP_Float ET;

    const typename Voronoi::Halfedge_handle he = voronoi.dual(edge);
    const Polygon poly1 = surroundingPolygon< Voronoi>(*(he->source()));
    const Polygon poly2 = surroundingPolygon< Voronoi>(*(he->target()));

//    BOOST_FOREACH(const Point p, poly.container())
//      std::cout << "(" << p << ") ";
//    std::cout << "point: " << point << std::endl;

    if(poly1.bounded_side(point) == CGAL::ON_UNBOUNDED_SIDE
        && poly2.bounded_side(point) == CGAL::ON_UNBOUNDED_SIDE)
    {
      typedef CGAL::Polytope_distance_d_traits_2< K, ET, K::RT> Traits;
      typedef CGAL::Polytope_distance_d< Traits> PolyDist;

      std::vector< Point> tmpPts;
      tmpPts.insert(tmpPts.end(), poly1.vertices_begin(), poly1.vertices_end());
      tmpPts.insert(tmpPts.end(), poly2.vertices_begin(), poly2.vertices_end());
      PolyDist polyDist(tmpPts.begin(), tmpPts.end(), &point, (&point) + 1);

      PolyDist::Coordinate_iterator it =
          polyDist.realizing_point_p_coordinates_begin();
      const double x = CGAL::to_double(*it);
      const double y = CGAL::to_double(*++it);
      const double denom = CGAL::to_double(*++it);

      return Point(x / denom, y / denom);
    }

    return point;
  }

}
}

#endif /* VORONOI_PATH_TRACER_DETAIL_H */
