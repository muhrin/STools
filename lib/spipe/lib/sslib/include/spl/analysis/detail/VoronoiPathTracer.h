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

template< typename LabelType>
  std::ostream &
  operator <<(std::ostream & os,
      const typename VoronoiPathTracer< LabelType>::Path & path)
  {
    BOOST_FOREACH(const typename VoronoiPathTracer< LabelType>::Point & pt,
        boost::make_iterator_range(path.verticesBegin(), path.verticesEnd()))
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
  class VoronoiPathTracer< LabelType>::GeneratePathVisitor : public std::binary_function<
      typename Voronoi::Halfedge_handle, const typename NextHalfedgeType::Value,
      void>
  {
  public:
    GeneratePathVisitor(Path * const path);
    void
    operator()(typename Voronoi::Halfedge_handle he,
        const typename NextHalfedgeType::Value nextType);
  private:
    bool
    isContinuation(const typename Delaunay::Edge & edge) const;

    Path * const myPath;
  };

template< typename LabelType>
  VoronoiPathTracer< LabelType>::GeneratePathVisitor::GeneratePathVisitor(
      Path * const path) :
      myPath(path)
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
      myPath->push_back(midpoint(dual), dual);

    if(nextType == NextHalfedgeType::IS_START)
      myPath->push_back(*myPath->begin());
    else if(nextType == NextHalfedgeType::IS_NULL)
    {
      // If we're at the end of the path it's either at the boundary of the data set
      // in which case the halfedge does not have a target (and we should do nothing)
      // or it's at a meeting vertex with another path in which case we should add
      // the vertex
      if(he->has_target())
        myPath->setBackVoronoiVertex(he->target());
    }
  }

template< typename LabelType>
  bool
  VoronoiPathTracer< LabelType>::GeneratePathVisitor::isContinuation(
      const typename Delaunay::Edge & edge) const
  {
    return !myPath->empty() && myPath->back().second == edge;
  }

template< typename LabelType>
  struct VoronoiPathTracer< LabelType>::Path
  {
  public:
    typedef std::pair< Point, typename Delaunay::Edge> value_type;
  private:
    typedef std::vector< value_type> Vertices;
    typedef utility::TakeFirst< const typename Vertices::value_type> TakeFirst;
    typedef utility::TakeSecond< const typename Vertices::value_type> TakeSecond;
  public:
    typedef typename Vertices::iterator iterator;
    typedef typename Vertices::const_iterator const_iterator;
    typedef boost::transform_iterator< TakeFirst, const_iterator> vertices_const_iterator;
    typedef boost::transform_iterator< TakeSecond, const_iterator> edges_const_iterator;

    void
    push_back(const Point & vtx, const typename Delaunay::Edge & spanningEdge)
    {
      myVertices.push_back(value_type(vtx, spanningEdge));
    }
    void
    push_back(const value_type & val)
    {
      myVertices.push_back(val);
    }
    iterator
    insert(iterator position, const value_type & val)
    {
      return myVertices.insert(position, val);
    }

    iterator
    begin()
    {
      return myVertices.begin();
    }
    iterator
    end()
    {
      return myVertices.end();
    }
    const_iterator
    begin() const
    {
      return myVertices.begin();
    }
    const_iterator
    end() const
    {
      return myVertices.end();
    }
    value_type &
    operator [](const size_t i)
    {
      return myVertices[i];
    }
    const value_type &
    operator [](const size_t i) const
    {
      return myVertices[i];
    }
    value_type &
    front()
    {
      return myVertices.front();
    }
    value_type &
    back()
    {
      return myVertices.back();
    }
    const value_type &
    front() const
    {
      return myVertices.front();
    }
    const value_type &
    back() const
    {
      return myVertices.back();
    }
    bool
    empty() const
    {
      return myVertices.empty();
    }

    vertices_const_iterator
    verticesBegin() const
    {
      return vertices_const_iterator(myVertices.begin());
    }

    vertices_const_iterator
    verticesEnd() const
    {
      return vertices_const_iterator(myVertices.end());
    }

    edges_const_iterator
    edgesBegin() const
    {
      return edges_const_iterator(myVertices.begin());
    }

    edges_const_iterator
    edgesEnd() const
    {
      return edges_const_iterator(myVertices.end());
    }

    void
    reverse();

    Path
    extract(const PossiblePath & possible) const
    {
      Path extracted;
      BOOST_FOREACH(const size_t & idx, possible)
        extracted.push_back(myVertices[idx]);
      return extracted;
    }

    size_t
    length() const
    {
      return myVertices.size();
    }

    bool
    isCircular() const
    {
      return !myVertices.empty() && myVertices.front() == myVertices.back();
    }

    typename Voronoi::Vertex_handle
    getFrontVoronoiVertex() const
    {
      return myFrontVoronoiVertex;
    }
    void
    setFrontVoronoiVertex(const typename Voronoi::Vertex_handle vtx)
    {
      myFrontVoronoiVertex = vtx;
    }
    typename Voronoi::Vertex_handle
    getBackVoronoiVertex() const
    {
      return myBackVoronoiVertex;
    }
    void
    setBackVoronoiVertex(const typename Voronoi::Vertex_handle vtx)
    {
      myBackVoronoiVertex = vtx;
    }

  private:
    typename Voronoi::Vertex_handle myFrontVoronoiVertex;
    typename Voronoi::Vertex_handle myBackVoronoiVertex;
    Vertices myVertices;
  };

template< typename LabelType>
  struct VoronoiPathTracer< LabelType>::TracingData
  {
    std::vector< Path> paths;
  };

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
    std::reverse(myVertices.begin(), myVertices.end());
    std::swap(myFrontVoronoiVertex, myBackVoronoiVertex);
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
  struct VoronoiPathTracer< LabelType>::PathInfo
  {
    std::vector< Line> fitLines;
    Path adjusted;
  };

template< typename LabelType>
  void
  VoronoiPathTracer< LabelType>::generatePaths(const Voronoi & voronoi,
      TracingData * const tracing, Arrangement * const arr) const
  {
    typedef std::vector< std::pair< size_t, bool> > MeetingInfo;
    typedef std::map< typename Voronoi::Vertex_handle, MeetingInfo> MeetingVertices;

    generatePathVertices(voronoi, tracing, arr);

    MeetingVertices meeting;
    std::vector< PathInfo> adjusted(tracing->paths.size());
    size_t idx = 0;
    BOOST_FOREACH(const Path & path, tracing->paths)
    {
      PathGraph paths = findStraightPaths(path, *tracing);
      //operator <<< LabelType>(std::cout, path);
      const PossiblePath & optimal = findOptimumPath(path, paths);
      adjusted[idx].fitLines = calculateLeastSquaresSubpaths(path, optimal);
      adjusted[idx].adjusted = generateVertexAdjustedPath(path, optimal,
          voronoi, adjusted[idx].fitLines);
      //operator <<< LabelType>(std::cout, path.extract(optimal));
      //operator <<< LabelType>(std::cout, adjusted[idx].adjusted);

      {
        const typename Voronoi::Vertex_handle front =
            path.getFrontVoronoiVertex();
        if(front != typename Voronoi::Vertex_handle() && front->is_valid())
          meeting[front].push_back(std::make_pair(idx, true));
      }
      {
        const typename Voronoi::Vertex_handle back =
            path.getBackVoronoiVertex();
        if(back != typename Voronoi::Vertex_handle() && back->is_valid())
          meeting[back].push_back(std::make_pair(idx, false));
      }

      ++idx;
    }

    typedef std::pair< size_t, bool> MeetingPair;
    BOOST_FOREACH(typename MeetingVertices::const_reference mv, meeting)
    {
      const Polygon poly = surroundingPolygon< Voronoi>(*mv.first);
      std::vector< Line> lines;
      BOOST_FOREACH(const MeetingPair & mi, mv.second)
      {
        if(!adjusted[mi.first].fitLines.empty())
        {
          if(mi.second)
            lines.push_back(adjusted[mi.first].fitLines.front());
          else
            lines.push_back(adjusted[mi.first].fitLines.back());
        }
      }
      const Point pt = meetingVertex(lines.begin(), lines.end(), poly);
      BOOST_FOREACH(const MeetingPair & mi, mv.second)
      {
        if(!adjusted[mi.first].fitLines.empty())
        {
          if(mi.second)
            adjusted[mi.first].adjusted.front() = std::make_pair(pt,
                typename Delaunay::Edge());
          else
            adjusted[mi.first].adjusted.back() = std::make_pair(pt,
                typename Delaunay::Edge());
        }
      }
    }

    BOOST_FOREACH(const PathInfo & info, adjusted)
    {
      operator <<< LabelType>(std::cout, info.adjusted);
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
        visitBoundaryHaledges(voronoi, he, GeneratePathVisitor(&path));
        if(!path.isCircular())
        {
          // Reverse the path and get the visitor to continue in the other direction
          path.reverse();
          he = he->twin();
          visitBoundaryHaledges(voronoi, he, GeneratePathVisitor(&path));
        }
        visited.insert(path.edgesBegin(), path.edgesEnd());
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

    const size_t n = path.length();
    PathGraph paths;
    bool pathStraight;
    for(size_t i = 0; i < n; ++i) // Start vertex
    {
      const Point & v_i = path[i].first;

      // Paths of length 1 are always possible
      for(size_t k = i + 1; k < std::min(i + 2, n); ++k)
        boost::add_edge(i, i + 1, paths);

      // Check longer paths
      for(size_t k = i + 2; k < n; ++k) // End vertex
      {
        const Point & v_k = path[k].first;
        const Segment ik(v_i, v_k);
        pathStraight = true;
        for(size_t j = i + 1; j < k; ++j) // Inbetween vertices
        {
          const Segment edgeSegment = segment(path[j].second);
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
    filterPossiblePaths(path, &paths);
    return paths;
  }

template< typename LabelType>
  void
  VoronoiPathTracer< LabelType>::filterPossiblePaths(const Path & path,
      PathGraph * const paths) const
  {
    typedef std::pair< PathGraph::edge_descriptor, bool> EdgeQuery;

    const size_t n = path.length();
    if(n < 4)
      return;

    EdgeQuery edge;
    if(path.isCircular())
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
    const size_t n = path.length();
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
    const size_t finalVertex = fullPath.length() - 1;
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

    const Segment v_ij(fullPath[subpath.first].first,
        fullPath[subpath.second].first);
    const double lenSq = CGAL::to_double(v_ij.squared_length());

    double sumSq = 0.0;
    for(size_t k = subpath.first; k < subpath.second; ++k)
      sumSq += CGAL::to_double(CGAL::squared_distance(fullPath[k].first, v_ij));

    return lenSq * 1 / static_cast< double>(n) * sumSq;
  }

template< typename LabelType>
  std::vector< typename VoronoiPathTracer< LabelType>::Line>
  VoronoiPathTracer< LabelType>::calculateLeastSquaresSubpaths(
      const Path & full, const PossiblePath & reduced) const
  {
    if(reduced.size() < 2)
      return std::vector< Line>();

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

      CGAL::linear_least_squares_fitting_2(full.verticesBegin() + sp.first,
          full.verticesBegin() + (sp.second + 1), lines[lineIdx],
          CGAL::Dimension_tag< 0>());
    }
    return lines;
  }

template< typename LabelType>
  typename VoronoiPathTracer< LabelType>::Path
  VoronoiPathTracer< LabelType>::generateVertexAdjustedPath(const Path & path,
      const PossiblePath & reduced, const Voronoi & voronoi,
      const std::vector< Line> & fitLines) const
  {
    if(path.length() == 1)
      return path;

    Path adjusted;

    // TODO: Make this work for circular paths

    // TODO: Intersect l1 and line supporting spanningEdges[0] to determine start point
    adjusted.push_back(path.front());

    // Only iterate from the second to the second last vertex indices
    const PossiblePath::const_iterator first = ++(reduced.begin());
    const PossiblePath::const_iterator end = --(reduced.end());

    size_t lineIdx = 0;
    for(PossiblePath::const_iterator it = first; it != end; ++it)
    {
      const typename CGAL::cpp11::result_of< K::Intersect_2
      (Line, Line)>::type result = CGAL::intersection(fitLines[lineIdx],
          fitLines[lineIdx + 1]);
      const Point * const intersection = boost::get< Point>(&*result);
      SSLIB_ASSERT(intersection);

      const typename Delaunay::Edge & spanningEdge = *(path.edgesBegin() + *it);

      // Constrain the intersection point to be 'near' the original vertex
      const Point constrained = constrainVertex(*intersection, voronoi,
          spanningEdge);

      adjusted.push_back(constrained, spanningEdge);
      ++lineIdx;
    }

    // TODO: Intersect l1 and line supporting spanningEdges[n - 1] to determine end point
    adjusted.push_back(path.back());

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

template< typename LabelType>
  template< typename LineIterator>
    typename VoronoiPathTracer< LabelType>::Point
    VoronoiPathTracer< LabelType>::meetingVertex(LineIterator begin,
        LineIterator beyond, const Polygon & bounding) const
    {
      typedef CGAL::Linear_algebraCd< K::FT> Linalg;
      typename Linalg::Matrix A(2, 2, 0.0);
      typename Linalg::Vector b(2, 0.0);
      for(LineIterator it = begin; it != beyond; ++it)
      {
        const typename K::Vector_2 ortho = it->to_vector().perpendicular(
            CGAL::COUNTERCLOCKWISE);
        const Point point = it->point();
        typename Linalg::Vector p(2);
        p[0] = point.x();
        p[1] = point.y();
        const K::FT lenSq = ortho.squared_length();
        typename Linalg::Matrix o(2, 1);
        o(0, 0) = ortho.x();
        o(1, 0) = ortho.y();
        const typename Linalg::Matrix oTrans = Linalg::transpose(o)
            * (1 / lenSq);
        const typename Linalg::Matrix oo = o * oTrans;
        A += oo;
        b += oo * p;
      }
      K::FT D;
      typename Linalg::Vector x;
      Linalg::linear_solver(A, b, x, D);

      Point rmsPt(x[0] / D, x[1] / D);

      // Now check that the isn't outside the boundary
      if(bounding.bounded_side(rmsPt) != CGAL::ON_UNBOUNDED_SIDE)
        return rmsPt;

      typedef CGAL::MP_Float ET;
      typedef CGAL::Polytope_distance_d_traits_2< K, ET, K::RT> Traits;
      typedef CGAL::Polytope_distance_d< Traits> PolyDist;

      PolyDist polyDist(bounding.vertices_begin(), bounding.vertices_end(),
          &rmsPt, (&rmsPt) + 1);

      PolyDist::Coordinate_iterator it =
          polyDist.realizing_point_p_coordinates_begin();
      const double xB = CGAL::to_double(*it);
      const double yB = CGAL::to_double(*++it);
      const double denom = CGAL::to_double(*++it);

      return Point(xB / denom, yB / denom);
    }

}
}

#endif /* VORONOI_PATH_TRACER_DETAIL_H */
