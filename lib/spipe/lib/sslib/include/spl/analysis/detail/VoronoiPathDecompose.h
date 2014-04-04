/*
 * VoronoiPathDecompose.h
 *
 *  Created on: Apr 4, 2014
 *      Author: Martin Uhrin
 */

#ifndef VORONOI_PATH_DECOMPOSE_DETAIL_H
#define VORONOI_PATH_DECOMPOSE_DETAIL_H

// INCLUDES ///////////////////
#include <set>

#include <boost/foreach.hpp>

#include "spl/SSLibAssert.h"
#include "spl/analysis/VoronoiPathUtility.h"

// FORWARD DECLARATIONS ///////

// DEFINITION ///////////////////////

namespace spl {
namespace analysis {
namespace detail {

struct NextHalfedgeType
{
  enum Value
  {
    IS_BOUNDARY, IS_NULL, IS_START
  };
};

template< typename VD>
  class GeneratePathVisitor : public std::binary_function<
      typename VD::Halfedge_handle, const typename NextHalfedgeType::Value, void>
  {
    typedef VD Voronoi;
    typedef VoronoiPathArrangement< VD> Arrangement;
    typedef typename Arrangement::Path Path;

  public:
    GeneratePathVisitor(Path * const path) :
        myPath(path)
    {
      SSLIB_ASSERT(myPath);
    }

    void
    operator()(typename Voronoi::Halfedge_handle he,
        const typename NextHalfedgeType::Value nextType)
    {
      myPath->push_back(he);
    }

  private:
    Path * const myPath;
  };

template< typename VD>
  typename VD::Halfedge_handle
  targetBoundaryHalfedge(const typename VD::Halfedge_handle & he)
  {
    typedef VD Voronoi;
    typedef typename VoronoiLabel< VD>::Type Label;

    if(!he->has_target())
      return typename Voronoi::Halfedge_handle();

    const typename BoundaryPair< Label>::Type & boundaryPair = getBoundaryPair<
        VD>(*he);

    typename Voronoi::Halfedge_handle next;
    const typename Voronoi::Vertex_handle target = he->target();
    const typename Voronoi::Halfedge_around_vertex_circulator start =
        target->incident_halfedges();
    typename Voronoi::Halfedge_around_vertex_circulator cl = start;
    do
    {
      if(typename Voronoi::Halfedge_handle(cl) != he
          && boundaryPair == getBoundaryPair< VD>(*cl))
      {
        next = cl->twin();
        break;
      }
      ++cl;
    } while(cl != start);
    return next;
  }

template< typename VD, typename Visitor>
  void
  visitBoundaryHaledges(const VD & voronoi,
      const typename VD::Halfedge_handle & start, Visitor visitor)
  {
    typedef VD Voronoi;
    typedef typename VoronoiLabel< VD>::Type Label;

    SSLIB_ASSERT(isBoundary< VD>(*start));

    const typename BoundaryPair< Label>::Type boundaryPair =
        getBoundaryPair< VD>(*start);

    typename Voronoi::Halfedge_handle next, he = start;
    typename NextHalfedgeType::Value nextType;
    do
    {
      next = targetBoundaryHalfedge< VD>(he);
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

template< typename VD>
  void
  decomposeInternalPaths(const VD & voronoi,
      VoronoiPathArrangement< VD> * const arrangement)
  {
    typedef VD Voronoi;
    typedef typename Voronoi::Delaunay_graph Delaunay;
    typedef VoronoiPathArrangement< VD> Arrangement;
    typedef typename Arrangement::Path Path;

    const Delaunay & delaunay = voronoi.dual();

    std::set< typename Delaunay::Edge> visited;
    typename Voronoi::Halfedge_handle he;
    typename Delaunay::Edge dual;
    for(typename Voronoi::Edge_iterator it = voronoi.edges_begin(), end =
        voronoi.edges_end(); it != end; ++it)
    {
      dual = it->dual();
      if(spansBoundary< Delaunay>(dual) && visited.find(dual) == visited.end()
          && visited.find(delaunay.mirror_edge(dual)) == visited.end())
      {
        Path path(voronoi);
        he = *it;
        // Trace the path out in one direction
        detail::visitBoundaryHaledges(voronoi, he,
            detail::GeneratePathVisitor< VD>(&path));
        if(!path.isCircular())
        {
          // Reverse the path and get the visitor to continue in the other direction
          path.reverse();
          he = targetBoundaryHalfedge< VD>(he->twin());
          if(he != typename Voronoi::Halfedge_handle())
            visitBoundaryHaledges(voronoi, he, GeneratePathVisitor< VD>(&path));
        }
        BOOST_FOREACH(const typename Path::Edge & edge,
            boost::make_iterator_range(path.edgesBegin(), path.edgesEnd()))
          visited.insert(edge.delaunayEdge());

        arrangement->insertPath(path);
      }
    }
  }

template< typename VD>
  void
  decomposeBoundaryPaths(const VD & voronoi,
      VoronoiPathArrangement< VD> * const arrangement)
  {
    typedef VD Voronoi;
    typedef typename Voronoi::Delaunay_graph Delaunay;
    typedef VoronoiPathArrangement< VD> Arrangement;
    typedef typename Arrangement::Path Path;

    const Delaunay & delaunay = voronoi.dual();

    typename Delaunay::Face_circulator start = delaunay.incident_faces(
        delaunay.infinite_vertex());
    typename Delaunay::Face_circulator cl = start;

    // First move the circulator to the first spanning edge
    // i.e. the first one that has an internal path that meets the
    // boundary if it exists (otherwise just end up at start)
    do
    {
      if(spansBoundary< Delaunay>(
          typename Delaunay::Edge(cl, cl->index(delaunay.infinite_vertex()))))
        break;
      --cl;
    } while(cl != start);

    // Set the start here and circulate from this point forward
    start = cl;

    Path current(voronoi);
    do
    {
      const typename Delaunay::Edge edge(cl,
          cl->index(delaunay.infinite_vertex()));
      current.push_back(edge);

      if(spansBoundary< Delaunay>(edge))
      {
        arrangement->insertBoundaryPath(current);
        // Start the new path and include this edge in it
        current = Path(voronoi);
        current.push_back(edge);
      }
      ++cl;
    } while(cl != start);

    const typename Delaunay::Edge edge(cl,
        cl->index(delaunay.infinite_vertex()));
    current.push_back(edge);

    arrangement->insertBoundaryPath(current);
  }

}

template< typename VD>
  void
  decomposePaths(const VD & voronoi,
      VoronoiPathArrangement< VD> * const arrangement)
  {
    detail::decomposeInternalPaths(voronoi, arrangement);
    //detail::decomposeBoundaryPaths(voronoi, arrangement);
  }

}
}

#endif /* VORONOI_PATH_DECOMPOSE_DETAIL_H */
