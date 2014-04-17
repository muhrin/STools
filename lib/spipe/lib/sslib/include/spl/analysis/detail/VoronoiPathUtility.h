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
#include <boost/range/iterator_range.hpp>

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
      // Check which way around we got the halfedge.  The labels will always
      // apply relative to the direction of the curve i.e. from
      // curve.source() -> curve.target():
      //
      // labels.first = halfedge
      // labels.second = halfedge.twin()
      if(e->curve().source() == e->source()->point())
      {
        e->data().label = labels.first;
        e->twin()->data().label = labels.second;
      }
      else
      {
        e->data().label = labels.second;
        e->twin()->data().label = labels.first;
      }
    }

    Map * const myMap;
    EdgeLabels myEdgeLabels;
    EdgeLabels mySplitEdgeLabels;
    boost::optional<
        std::pair< typename Map::Vertex_handle, typename Map::Vertex_handle> > myUpcomingEdge;
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
    typedef detail::MapObserver< Map, Label> EdgeLabeller;

    typedef std::map< typename VD::Vertex_handle, typename Map::Vertex_handle> MeetingVertices;
    typedef std::map< typename Delaunay::Edge, typename Map::Vertex_handle> BoundaryVertices;
    typedef std::map< const Path *, std::vector< typename Map::Vertex_handle> > PathVertices;

    void
    placePathEdges(const typename PathArrangement::Path & path, Map * const map)
    {
      typedef typename K::Segment_2 Segment;

      Point p0, p1;

      typename Delaunay::Edge dgEdge;
      EdgeLabeller labeller(map);

      // Grab the edge labels from the first edge
      if(path.numEdges() != 0)
      {
        const typename Path::Edge & edge = path.edgeFront();
        labeller.setEdgeLabels(
            typename EdgeLabeller::EdgeLabels(edge.leftLabel(),
                edge.rightLabel()));
      }

      BOOST_FOREACH(const typename Path::Edge & edge,
          boost::make_iterator_range(path.edgesBegin(), path.edgesEnd()))
      {
        p0 = convert(path.vertex(edge.source()).point());
        p1 = convert(path.vertex(edge.target()).point());
        if(p0 != p1)
          CGAL::insert(*map, Segment(p0, p1));
      }
    }

    void
    createBoundary(const PathArrangement & arrangement, Map * const map)
    {
      // Here we perform a counter-clockwise circulation of the infinite
      // vertex.  This corresponds to a clockwise circulation of the convex
      // hull of the Delaunay triangulation.

      typedef VD Voronoi;
      typedef typename Voronoi::Delaunay_graph Delaunay;
      typedef typename Delaunay::Edge Edge;
      typedef typename Delaunay::Face_circulator FaceCirculator;
      typedef typename K::Segment_2 Segment;
      typedef typename EdgeLabeller::EdgeLabels EdgeLabels;

      const Voronoi & voronoi = arrangement.getVoronoi();
      const Delaunay & delaunay = voronoi.dual();

      const typename Delaunay::Vertex_handle infiniteVertex =
          delaunay.infinite_vertex();

      FaceCirculator start = delaunay.incident_faces(infiniteVertex);
      FaceCirculator cl = start;
      do
      {
        if(spansBoundary< Delaunay>(Edge(cl, cl->index(infiniteVertex))))
          break;
        ++cl;
      } while(cl != start);
      start = cl; // Start at the newly found boundary edge (or the original start)

      EdgeLabeller labeller(map);
      labeller.setEdgeLabels(
          EdgeLabels(boost::optional< Label>(),
              cl->vertex(delaunay.ccw(cl->index(infiniteVertex)))->info()));
      Point p0, p1;
      const typename PathArrangement::BoundaryVerticesConst & boundaryVertices =
          arrangement.getBoundaryVertices();
      typename PathArrangement::BoundaryVerticesConst::const_iterator it;
      do
      {
        const Edge edge(cl, cl->index(infiniteVertex));

        if(spansBoundary< Delaunay>(edge))
        {
          // Viewing a vertical edge with the infinite vertex (i) to its right
          // that span a boundary it looks like this:
          // .|  <- ccw(i)
          // -|  <- The boundary edge that intersects the convex hull
          // .|  <- cw(i)

          it = boundaryVertices.find(edge);
          SSLIB_ASSERT(it != boundaryVertices.end());

          // Create the first edge up to the intersection point on the boundary
          p0 = convert(cl->vertex(delaunay.ccw(edge.second))->point());
          p1 =
              it->second.second == 0 ?
                  convert(it->second.first->vertexFront().point()) :
                  convert(it->second.first->vertexBack().point());

          // Insert the edge segment into the arrangement
          if(p0 != p1)
            CGAL::insert(*map, Segment(p0, p1));

          labeller.setEdgeLabels(
              EdgeLabels(boost::optional< Label>(),
                  cl->vertex(delaunay.cw(edge.second))->info()));

          // Now create the second edge from the intersection point to the end
          // of the Delaunay edge
          p0 = p1;
          p1 = convert(cl->vertex(delaunay.cw(edge.second))->point());
          // Insert the edge segment into the arrangement
          if(p0 != p1)
            CGAL::insert(*map, Segment(p0, p1));
        }
        else
        {
          p0 = convert(cl->vertex(delaunay.ccw(edge.second))->point());
          p1 = convert(cl->vertex(delaunay.cw(edge.second))->point());
          // Insert the edge segment into the arrangement
          if(p0 != p1)
            CGAL::insert(*map, Segment(p0, p1));
        }

        ++cl;
      } while(cl != start);

      const Edge edge(cl, cl->index(infiniteVertex));
    }

    void
    populateFaceLabels(Map * const map) const
    {
      BOOST_FOREACH(typename Map::Face & f,
          boost::make_iterator_range(map->faces_begin(), map->faces_end()))
      {
        if(f.has_outer_ccb())
        {
          const typename Map::Ccb_halfedge_circulator start = f.outer_ccb();
          typename Map::Ccb_halfedge_circulator cl = start;
          do
          {
            if(cl->data().label)
            {
              f.data().label = cl->data().label;
              break;
            }
            ++cl;
          } while(cl != start);
        }
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
        edge.first->vertex((edge.second + 2) % 3)->info(),
        edge.first->vertex((edge.second + 1) % 3)->info());
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
    return edge.first->vertex((edge.second + 2) % 3)->info()
        != edge.first->vertex((edge.second + 1) % 3)->info();
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
    Map map;

    Builder builder;

    std::cout << "CONSTRUCTING ARRANGEMENT\n";

    // Connect the edges
    BOOST_FOREACH(const typename Builder::Path & path,
        boost::make_iterator_range(pathArrangement.pathsBegin(), pathArrangement.pathsEnd()))
      builder.placePathEdges(path, &map);

    builder.createBoundary(pathArrangement, &map);
    builder.populateFaceLabels(&map);

    return map;
  }

}
}

#endif /* VORONOI_PATH_UTILITY_DETAIL_H */
