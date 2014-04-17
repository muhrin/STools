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
        myIncoming[boost::target(e, g)].push_back(boost::source(e, g));
      }

    template< class Graph>
      void
      finish_vertex(Vertex u, Graph & g)
      {
//        if(u == myFinishVertex)
//          throw 0;
      }

  private:
    IncomingMap myIncoming;
    Vertex myFinishVertex;
  };

template< typename LabelType>
  class VoronoiPathTracer< LabelType>::DirectionChecker
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

template< typename LabelType>
  struct VoronoiPathTracer< LabelType>::TracingData
  {
    TracingData(const Voronoi & voronoi) :
        pathArrangement(voronoi)
    {
    }
    VoronoiPathArrangement< Voronoi> pathArrangement;
  };

template< typename LabelType>
  typename VoronoiPathTracer< LabelType>::Map
  VoronoiPathTracer< LabelType>::generateMap(const Voronoi & voronoi) const
  {
    TracingData tracingData(voronoi);
    const PathArrangement & pathArrangement = generatePaths(voronoi,
        &tracingData);

    return analysis::toMap< LabelType>(pathArrangement);
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
    const Path * orig;
    Path adjusted;
  };

template< typename LabelType>
  typename VoronoiPathTracer< LabelType>::PathArrangement
  VoronoiPathTracer< LabelType>::generatePaths(const Voronoi & voronoi,
      TracingData * const tracing) const
  {
    decomposePaths(voronoi, &tracing->pathArrangement);
    //return tracing->pathArrangement;

    PathArrangement pathArr(voronoi);

    BOOST_FOREACH(const Path & path,
        boost::make_iterator_range(tracing->pathArrangement.pathsBegin(), tracing->pathArrangement.pathsEnd()))
    {
      const PathGraphInfo & paths = findStraightPathsInternal(path,
          tracing->pathArrangement);
      const PossiblePath & optimal = findOptimumPath(path, paths);

      Path optimalPath = extractReducedPath(path, optimal);
      adjustVertices(&optimalPath);

      pathArr.insertPath(optimalPath);
    }

    optimiseBoundaryVertices(&pathArr);
    //optimiseMeetingVertices(&pathArr);

    return pathArr;
  }

template< typename LabelType>
  typename VoronoiPathTracer< LabelType>::PathGraphInfo
  VoronoiPathTracer< LabelType>::findStraightPathsInternal(const Path & path,
      const PathArrangement & arr) const
  {
    const ptrdiff_t n = static_cast< ptrdiff_t>(path.numVertices());
    PathGraphInfo paths;
    // Populate the graph with the vertices for this path
    typename boost::property_map< PathGraph, PathVertexIndexType>::type index =
        boost::get(PathVertexIndexType(), paths.graph);
    for(ptrdiff_t i = -1; i <= n; ++i)
    {
      const typename PathGraph::vertex_descriptor vtx = boost::add_vertex(
          paths.graph);
      paths.vertexMap[i] = vtx;
      boost::put(index, vtx, i);
    }

    // TODO: Add circular path support
    arma::Mat<int> straight(n + 2, n + 2);
    straight.zeros();
    for(ptrdiff_t i = 0; i < n - 1; ++i) // Start vertex
    {
      for(ptrdiff_t k = i + 1; k < n; ++k) // End vertex
      {
        straight(i + 1, k + 1) = isStraight(path, i, k);
      }
    }

    if(!path.isCircular())
    {
      bool pathStraight;
      const typename PathArrangement::MeetingVerticesConst & meeting =
          arr.getMeetingVertices();
      std::pair< typename PathArrangement::MeetingVerticesConst::const_iterator,
          typename PathArrangement::MeetingVerticesConst::const_iterator> meetingRange;

      // Vertex before the start meeting vertex
      if(path.vertexFront().isBoundary())
      {
        for(ptrdiff_t i = 0; i < n; ++i)
        {
          if(boost::edge(paths.vertexMap[0], paths.vertexMap[i], paths.graph).second)
          {
//            boost::add_edge(paths.vertexMap[-1], paths.vertexMap[i],
//                paths.graph);
            straight(0, i + 1) = true;
          }
        }
      }
      else
      {
        // Paths of length 1-2 are always possible
        for(ptrdiff_t i = 0; i < std::min(static_cast< ptrdiff_t>(2), n); ++i)
        {
//          boost::add_edge(paths.vertexMap[-1], paths.vertexMap[i], paths.graph);
          straight(0, i + 1) = true;
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
                path.vertex(0).point()
                    - vtx.second.first->vertex(vtx.second.second).point());
            dirs.update(path.vertex(1).point() - path.vertex(0).point());

            if(!isStraight(e1.delaunayEdge(), e2.delaunayEdge(), path, 0, i - 2,
                &dirs))
            {
              pathStraight = false;
              break;
            }
          }
//          if(pathStraight)
//            boost::add_edge(paths.vertexMap[-1], paths.vertexMap[i],
//                paths.graph);
          straight(0, i + 1) = pathStraight;

        }
      }

      // Vertex after the end meeting vertex
      if(n > 1)
      {
        if(path.vertexBack().isBoundary())
        {
          for(ptrdiff_t i = 0; i < n; ++i)
          {
            if(boost::edge(paths.vertexMap[i], paths.vertexMap[n - 1],
                paths.graph).second)
            {
//              boost::add_edge(paths.vertexMap[i], paths.vertexMap[n],
//                  paths.graph);
              straight(i + 1, n + 1) = true;
            }
          }
        }
        else
        {
          // Paths of length 1-2 are always possible
          for(ptrdiff_t i = n - 1;
              i >= std::max(n - 2, static_cast< ptrdiff_t>(0)); --i)
          {
//            boost::add_edge(paths.vertexMap[i], paths.vertexMap[n],
//                paths.graph);
            straight(i + 1, n + 1) = true;
          }

          meetingRange = meeting.equal_range(
              path.vertex(n - 1).voronoiVertex());

          for(ptrdiff_t i = n - 3; i >= 0; --i) // End vertex
          {
            const typename Path::Edge & e1 = path.edge(i);

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
//            if(pathStraight)
//              boost::add_edge(paths.vertexMap[i], paths.vertexMap[n],
//                  paths.graph);
            straight(i + 1, n + 1) = pathStraight;
          }
        }
      }

      if(!path.vertexFront().isBoundary() && n > 1
          && !path.vertexBack().isBoundary())
      {
        meetingRange = meeting.equal_range(path.vertexFront().voronoiVertex());
        std::pair<
            typename PathArrangement::MeetingVerticesConst::const_iterator,
            typename PathArrangement::MeetingVerticesConst::const_iterator> meetingRange2 =
            meeting.equal_range(path.vertexBack().voronoiVertex());
        pathStraight = true;
        BOOST_FOREACH(typename PathArrangement::MeetingVerticesConst::const_reference vtx,
            meetingRange)
        {
          const typename Path::Edge & e1 =
              vtx.second.second == 0 ?
                  vtx.second.first->edgeFront() : vtx.second.first->edgeBack();

          BOOST_FOREACH(typename PathArrangement::MeetingVerticesConst::const_reference vtx2,
              meetingRange2)
          {
            const typename Path::Edge & e2 =
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
//        if(pathStraight)
//          boost::add_edge(paths.vertexMap[0], paths.vertexMap[n], paths.graph);
        straight(1, n + 1) = pathStraight;
      }
    }

    for(ptrdiff_t i = -1; i < n + 1; ++i) // Start vertex
    {
      for(ptrdiff_t k = i + 1; k < n + 1; ++k) // End vertex
      {
        bool allStraight = true;
        for(ptrdiff_t l = i; l < k; ++l) // Start
        {
          for(ptrdiff_t m = l + 1; m <= k; ++m) // End
          {
            if(!straight(l + 1, m + 1))
            {
              allStraight = false;
              break;
            }
          }
          if(!allStraight)
            break;
        }
        if(allStraight)
          boost::add_edge(paths.vertexMap[i], paths.vertexMap[k], paths.graph);
        else
          break;
      }
    }

    std::cout << path.numEdges() << "\n";
    BOOST_FOREACH(const typename PathGraph::edge_descriptor & edge, boost::edges(paths.graph))
    {
      std::cout << boost::get(index, boost::source(edge, paths.graph)) << "->"
          << boost::get(index, boost::target(edge, paths.graph)) << "\n";
    }
    std::cout << "\n" << std::endl;

    filterPossiblePaths(path, &paths);

    std::cout << path.numEdges() << "\n";
    BOOST_FOREACH(const typename PathGraph::edge_descriptor & edge, boost::edges(paths.graph))
    {
      std::cout << boost::get(index, boost::source(edge, paths.graph)) << "->"
          << boost::get(index, boost::target(edge, paths.graph)) << "\n";
    }
    std::cout << "\n" << std::endl;

    return paths;
  }

template< typename LabelType>
  bool
  VoronoiPathTracer< LabelType>::isStraight(const Path & path, const size_t i,
      const size_t k) const
  {
    SSLIB_ASSERT(i < path.numVertices());
    SSLIB_ASSERT(k < path.numVertices());
    SSLIB_ASSERT(i <= k);

    // Paths of length 1 & 2 are always straight
    if((i - k) <= 2 || (k - i) <= 2)
      return true;

    // The start and end Delaunay edges
    const typename Delaunay::Edge & dgE1 = path.edge(i).delaunayEdge();
    const typename Delaunay::Edge & dgE2 = path.edge(k - 1).delaunayEdge();

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
    dirs.update(path.vertex(i + 1).point() - path.vertex(i).point());

    bool pathStraight;
    for(ptrdiff_t j = i + 2; j < k; ++j) // Inbetween vertices
    {
      pathStraight = false;
      const typename Path::Edge & edge = path.edge(j - 1);
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
    dirs.update(path.vertex(k).point() - path.vertex(k - 1).point());
    if(dirs.numDirections() == 4)
      pathStraight = false;

    return pathStraight;
  }

template< typename LabelType>
  bool
  VoronoiPathTracer< LabelType>::isStraight(
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

template< typename LabelType>
  void
  VoronoiPathTracer< LabelType>::filterPossiblePaths(const Path & path,
      PathGraphInfo * const paths) const
  {
    typedef std::pair< typename PathGraph::edge_descriptor, bool> EdgeQuery;

    const ptrdiff_t n = static_cast< ptrdiff_t>(path.numVertices());
    if(n < 3)
      return;

    // TODO: Implement a version for circular paths

    EdgeQuery edge;
    std::vector< typename PathGraph::edge_descriptor> toRemove;
    ptrdiff_t iBack, jForward;
    for(ptrdiff_t i = 0; i < n; ++i)
    {
      iBack = i - 1;

      for(ptrdiff_t j = i + 2; j < n; ++j)
      {
        edge = boost::edge(paths->vertexMap[i], paths->vertexMap[j],
            paths->graph);
        if(edge.second)
        {
          jForward = j + 1;

          if(!boost::edge(paths->vertexMap[iBack], paths->vertexMap[jForward],
              paths->graph).second)
            toRemove.push_back(edge.first);
        }
      }
    }
    BOOST_FOREACH(const typename PathGraph::edge_descriptor & e, toRemove)
      paths->graph.remove_edge(e);
  }

template< typename LabelType>
  typename VoronoiPathTracer< LabelType>::PossiblePath
  VoronoiPathTracer< LabelType>::findOptimumPath(const Path & path,
      const PathGraphInfo & paths) const
  {
    const ptrdiff_t n = static_cast< ptrdiff_t>(path.numVertices());
    if(n == 0)
      return PossiblePath();
    else if(n == 1)
    {
      PossiblePath poss;
      poss.insert(0);
      return poss;
    }

    const typename PathGraph::vertex_descriptor source =
        paths.vertexMap.find(0)->second;
    const typename PathGraph::vertex_descriptor target = paths.vertexMap.find(
        n - 1)->second;
    IncomingMap incoming;
    // Make the source it's own root to allow general algorithms
    // i.e. not have to conditionally check for source and do special case
    incoming[source].push_back(source);

    RecordIncoming< IncomingMap &, typename PathGraph::vertex_descriptor> visitor(
        incoming, target);
    try
    {
      boost::breadth_first_search(paths.graph, source, boost::visitor(visitor));
    }
    catch(const int)
    {
    }

    std::vector< PossiblePath> possiblePaths;
    generatePossiblePaths(path, incoming, paths, &possiblePaths);

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
      const IncomingMap & incoming, const PathGraphInfo & pathsInfo,
      std::vector< PossiblePath> * const possiblePaths) const
  {
    if(incoming.empty())
      return;

    // Get the mapping from graph vertices to path index
    const typename boost::property_map< PathGraph, PathVertexIndexType>::const_type index =
        boost::get(PathVertexIndexType(), pathsInfo.graph);

    PossiblePath & currentPath = *possiblePaths->insert(possiblePaths->end(),
        PossiblePath());
    const typename PathGraph::vertex_descriptor startVertex =
        pathsInfo.vertexMap.find(0)->second;
    const typename PathGraph::vertex_descriptor finalVertex =
        pathsInfo.vertexMap.find(fullPath.numVertices() - 1)->second;

    currentPath.insert(fullPath.numVertices() - 1);

    generatePossiblePaths(incoming, finalVertex, startVertex, index,
        &currentPath, possiblePaths);
  }

template< typename LabelType>
  void
  VoronoiPathTracer< LabelType>::generatePossiblePaths(
      const IncomingMap & incoming,
      const typename PathGraph::vertex_descriptor currentVertex,
      const typename PathGraph::vertex_descriptor targetVertex,
      const typename boost::property_map< PathGraph, PathVertexIndexType>::const_type & index,
      PossiblePath * const currentPath,
      std::vector< PossiblePath> * const possiblePaths) const
  {
    if(currentVertex == targetVertex)
      return;

    const typename IncomingMap::const_iterator it = incoming.find(
        currentVertex);
    SSLIB_ASSERT(it != incoming.end());
    SSLIB_ASSERT(!it->second.empty());

    // Make a copy of the path so far
    const PossiblePath pathSoFar = *currentPath;

    // Continue the current path
    const typename PathGraph::vertex_descriptor predecessor = it->second[0];
    currentPath->insert(boost::get(index, predecessor));
    generatePossiblePaths(incoming, predecessor, targetVertex, index,
        currentPath, possiblePaths);

    // Do any branching paths
    for(size_t i = 1; i < it->second.size(); ++i)
    {
      const size_t predecessor = it->second[i];
      PossiblePath & splitPath = *possiblePaths->insert(possiblePaths->end(),
          pathSoFar);
      splitPath.insert(predecessor);
      generatePossiblePaths(incoming, predecessor, targetVertex, index,
          &splitPath, possiblePaths);
    }
  }

template< typename LabelType>
  typename VoronoiPathTracer< LabelType>::WeightMap
  VoronoiPathTracer< LabelType>::calculatePenalties(const Path & fullPath,
      const PathGraph & shortestPaths) const
  {
    WeightMap weights;

    BOOST_FOREACH(const typename PathGraph::edge_descriptor & edge, boost::edges(shortestPaths))
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

    const Segment v_ij(fullPath.vertex(subpath.first).point(),
        fullPath.vertex(subpath.second).point());
    const double lenSq = CGAL::to_double(v_ij.squared_length());

    double sumSq = 0.0;
    for(size_t k = subpath.first; k < subpath.second; ++k)
      sumSq += CGAL::to_double(
          CGAL::squared_distance(fullPath.vertex(k).point(), v_ij));

    return lenSq * (1.0 / static_cast< double>(n)) * sumSq;
  }

template< typename LabelType>
  typename VoronoiPathTracer< LabelType>::LineAndCoM
  VoronoiPathTracer< LabelType>::calculateLeastSquaresLine(const Path & full,
      const size_t idx0, const size_t idx1) const
  {
    SSLIB_ASSERT(idx0 < full.numVertices() && idx1 < full.numVertices());

    std::vector< Point> pts;
    for(typename Path::VertexConstIterator it = full.verticesBegin() + idx0,
        end = full.verticesBegin() + idx1 + 1; it != end; ++it)
      pts.push_back(it->point());

    LineAndCoM line;
    CGAL::linear_least_squares_fitting_2(pts.begin(), pts.end(), line.first,
        line.second, CGAL::Dimension_tag< 0>());

    return line;
  }

template< typename LabelType>
  typename CGAL::Linear_algebraCd< typename VoronoiPathTracer< LabelType>::K::FT>::Matrix
  VoronoiPathTracer< LabelType>::calculateQuadraticForm(
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

template< typename LabelType>
  std::vector< typename VoronoiPathTracer< LabelType>::Line>
  VoronoiPathTracer< LabelType>::calculateLeastSquaresSubpaths(
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

template< typename LabelType>
  typename VoronoiPathTracer< LabelType>::Path
  VoronoiPathTracer< LabelType>::extractReducedPath(const Path & full,
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

template< typename LabelType>
  void
  VoronoiPathTracer< LabelType>::placeMeetingVertices(
      TracingData * const tracing) const
  {
    typedef std::multimap< typename Voronoi::Vertex_handle,
        std::pair< const Path *, size_t> > MeetingMap;
    for(typename MeetingMap::const_iterator it =
        tracing->meetingVertices.begin(), end = tracing->meetingVertices.end();
        it != end; it = tracing->meetingVertices.upper_bound(it->first))
    {
      tracing->placedMeetingVertices[it->first] =
          tracing->arrangement.insert_in_face_interior(
              it->second.first->vertex(it->second.second).point(),
              tracing->arrangement.unbounded_face());
    }
  }

template< typename LabelType>
  void
  VoronoiPathTracer< LabelType>::adjustVertices(Path * const path) const
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

template< typename LabelType>
  void
  VoronoiPathTracer< LabelType>::optimiseBoundaryVertices(
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

template< typename LabelType>
  void
  VoronoiPathTracer< LabelType>::optimiseMeetingVertices(
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

template< typename LabelType>
  typename VoronoiPathTracer< LabelType>::Point
  VoronoiPathTracer< LabelType>::joinLines(
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

template< typename LabelType>
  typename VoronoiPathTracer< LabelType>::Polygon
  VoronoiPathTracer< LabelType>::surroundingBoundary(
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

template< typename LabelType>
  typename VoronoiPathTracer< LabelType>::K::FT
  VoronoiPathTracer< LabelType>::quadDist(const Point & p,
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
