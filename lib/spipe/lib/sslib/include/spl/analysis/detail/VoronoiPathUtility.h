/*
 * VoronoiPathUtility.h
 *
 *  Created on: Feb 18, 2014
 *      Author: Martin Uhrin
 */

#ifndef VORONOI_PATH_UTILITY_DETAIL_H
#define VORONOI_PATH_UTILITY_DETAIL_H

// INCLUDES ///////////////////

#include "spl/SSLibAssert.h"

#include <map>

#include <boost/foreach.hpp>

#include <CGAL/Arr_observer.h>
#include <CGAL/Point_2.h>
#include <CGAL/Segment_2.h>

// FORWARD DECLARATIONS ///////

// DEFINITION ///////////////////////

namespace spl {
namespace analysis {
namespace detail {

template< typename Map, typename Label>
  class MapObserver : CGAL::Arr_observer< Map>
  {
  public:
    typedef std::pair< boost::optional< Label>, boost::optional< Label> > EdgeLabels;

    MapObserver(Map * const map) :
        myMap(map)
    {
      this->attach(*myMap);
    }
    virtual
    ~MapObserver()
    {
      this->detach();
    }

    void
    setEdgeLabels(const EdgeLabels & labels)
    {
      myEdgeLabels = labels;
    }

  private:
    virtual void
    after_create_edge(typename Map::Halfedge_handle e)
    {
      assignLabels(myEdgeLabels, e);
    }

    virtual void
    before_split_edge(typename Map::Halfedge_handle e,
        typename Map::Vertex_handle v,
        const typename Map::X_monotone_curve_2 & c1,
        const typename Map::X_monotone_curve_2 & c2)
    {
      // Save the labels from the edge about to be split
      mySplitEdgeLabels.first = e->data().label;
      mySplitEdgeLabels.second = e->twin()->data().label;
    }

    virtual void
    after_split_edge(typename Map::Halfedge_handle e1,
        typename Map::Halfedge_handle e2)
    {
      // Assign the labels from the edge before splitting to these new edges
      assignLabels(mySplitEdgeLabels, e1);
      assignLabels(mySplitEdgeLabels, e2);
    }

    void
    assignLabels(const EdgeLabels & labels,
        typename Map::Halfedge_handle e) const
    {
      e->data().label = labels.first;
      e->twin()->data().label = labels.second;
    }

    Map * const myMap;
    EdgeLabels myEdgeLabels;
    EdgeLabels mySplitEdgeLabels;
  };

template< typename Map, typename Label, typename VD>
  struct MapBuilder
  {
    typedef VD Voronoi;
    typedef typename VD::Delaunay_graph Delaunay;
    typedef typename Map::Geometry_traits_2 K;
    typedef typename Map::Point_2 Point;
    typedef VoronoiPathArrangement< VD> PathArrangement;
    typedef typename PathArrangement::Path Path;

    typedef std::map< typename VD::Vertex_handle, typename Map::Vertex_handle> MeetingVertices;
    typedef std::map< typename Delaunay::Edge, typename Map::Vertex_handle> BoundaryVertices;
    typedef std::map< const Path *, std::vector< typename Map::Vertex_handle> > PathVertices;

    void
    placePathEdges(const typename PathArrangement::Path & path, Map * const map)
    {
      typedef typename K::Segment_2 Segment;

      Point p0, p1;

      typename Delaunay::Edge dgEdge;

      BOOST_FOREACH(const typename Path::Edge & edge,
          boost::make_iterator_range(path.edgesBegin(), path.edgesEnd()))
      {
        p0 = convert(path.vertex(edge.source()).point());
        p1 = convert(path.vertex(edge.target()).point());

        CGAL::insert(*map, Segment(p0, p1));
      }
    }

    Point
    convert(const typename Path::Point & pt)
    {
      return Point(pt.x(), pt.y());
    }

    MeetingVertices meeting;
    BoundaryVertices boundary;
    PathVertices pathVertices;
  };
}

template< class VD>
  typename BoundaryPair< typename VoronoiLabel< VD>::Type>::Type
  getBoundaryPair(const typename VD::Halfedge & he)
  {
    return getSpanningPair< typename VD::Delaunay_graph>(he.dual());
  }

template< class DG>
  typename BoundaryPair< typename DelaunayLabel< DG>::Type>::Type
  getSpanningPair(const typename DG::Edge & edge)
  {
    return typename BoundaryPair< typename DelaunayLabel< DG>::Type>::Type(
        edge.first->vertex((edge.second + 1) % 3)->info(),
        edge.first->vertex((edge.second + 2) % 3)->info());
  }

template< class VD>
  bool
  isBoundary(const typename VD::Halfedge & he)
  {
    return spansBoundary< typename VD::Delaunay_graph>(he.dual());
  }

template< class DG>
  bool
  spansBoundary(const typename DG::Edge & edge)
  {
    return edge.first->vertex((edge.second + 1) % 3)->info()
        != edge.first->vertex((edge.second + 2) % 3)->info();
  }

template< typename VD>
  CGAL::Polygon_2< typename VD::Delaunay_geom_traits>
  delaunayDomain(const typename VD::Vertex_handle & vtx, const VD & voronoi)
  {
    typedef typename VD::Delaunay_graph Delaunay;

    SSLIB_ASSERT(vtx->is_valid());

    const Delaunay & delaunay = voronoi.dual();
    CGAL::Polygon_2< typename VD::Delaunay_geom_traits> poly;

    typename VD::Halfedge_around_vertex_circulator start =
        voronoi.incident_halfedges(vtx);
    typename VD::Halfedge_around_vertex_circulator cl = start;
    typename Delaunay::Edge dgEdge;
    do
    {
      dgEdge = cl->dual();
      poly.push_back(
          dgEdge.first->vertex(delaunay.ccw(dgEdge.second))->point());

      // Move on twice to get to the next edge (not halfedge)
      ++cl;
      ++cl;
    } while(cl != start);

    return poly;
  }

template< typename Label, typename VD>
  typename MapArrangement< CGAL::Exact_predicates_exact_constructions_kernel,
      Label>::Arrangement
  toMap(const VoronoiPathArrangement< VD> & pathArrangement)
  {
    typedef typename MapArrangement<
        CGAL::Exact_predicates_exact_constructions_kernel, Label>::Arrangement Map;
    typedef detail::MapBuilder< Map, Label, VD> Builder;
    typedef detail::MapObserver< Map, Label> MapObserver;
    Map map;

    MapObserver observer(&map);
    Builder builder;

    // Connect the edges
    BOOST_FOREACH(const typename Builder::Path & path,
        boost::make_iterator_range(pathArrangement.pathsBegin(), pathArrangement.pathsEnd()))
    {
      typename MapObserver::EdgeLabels edgeLabels;
      if(path.numEdges() != 0)
      {
        const typename Builder::Path::Edge & edge = path.edge(0);
        edgeLabels.first = edge.leftLabel();
        edgeLabels.second = edge.rightLabel();
      }
      observer.setEdgeLabels(edgeLabels);
      builder.placePathEdges(path, &map);
    }

    typename MapObserver::EdgeLabels edgeLabels;
    observer.setEdgeLabels(edgeLabels);
    BOOST_FOREACH(const typename Builder::Path & path,
        boost::make_iterator_range(pathArrangement.boundaryPathsBegin(), pathArrangement.boundaryPathsEnd()))
    {
      typename MapObserver::EdgeLabels edgeLabels;
      if(path.numEdges() != 0)
      {
        const typename Builder::Path::Edge & edge = path.edge(0);
        edgeLabels.first = edge.leftLabel();
        edgeLabels.second = edge.rightLabel();
      }
      observer.setEdgeLabels(edgeLabels);
      builder.placePathEdges(path, &map);
    }

    return map;
  }

}
}

#endif /* VORONOI_PATH_UTILITY_DETAIL_H */
