/*
 * VoronoiPath.h
 *
 *  Created on: Mar 18, 2014
 *      Author: Martin Uhrin
 */

#ifndef VORONOI_PATH_H
#define VORONOI_PATH_H

// INCLUDES ////////////
#include "spl/SSLib.h"

#ifdef SPL_WITH_CGAL

#include <vector>

#include <CGAL/Linear_algebraCd.h>

#include "spl/analysis/VoronoiPathUtility.h"

// FORWARD DECLARATIONS ///////

// DEFINITION ///////////////////////

namespace spl {
namespace analysis {

template< typename VD>
  class VoronoiPath
  {
    typedef typename VoronoiLabel< VD>::Type Label;
  public:
    typedef VD Voronoi;
    typedef typename Voronoi::Delaunay_graph Delaunay;
    typedef typename Voronoi::Adaptation_traits::Point_2 Point;
    typedef typename Voronoi::Delaunay_geom_traits GeomTraits;
    typedef CGAL::Polygon_2< GeomTraits> Polygon;
    typedef CGAL::Segment_2< GeomTraits> Segment;

    typedef std::pair< Point, typename Delaunay::Edge> value_type;

    class Vertex;
    class Edge;
  private:
    typedef std::vector< Vertex> Vertices;
    typedef std::vector< Edge> Edges;
  public:
    typedef typename Vertices::iterator VertexIterator;
    typedef typename Vertices::const_iterator VertexConstIterator;
    typedef typename Edges::iterator EdgeIterator;
    typedef typename Edges::const_iterator EdgeConstIterator;

    VoronoiPath();
    explicit VoronoiPath(const Voronoi & voronoi);
    VoronoiPath(const VoronoiPath & path);

    size_t
    push_back(const typename Voronoi::Halfedge_handle & he);
    size_t
    push_back(const Vertex & vtx1, const Vertex & vtx2,
        const typename Delaunay::Edge & dgEdge);
    size_t
    close(const typename Voronoi::Halfedge_handle & he);

    VertexConstIterator
    verticesBegin() const;
    VertexConstIterator
    verticesEnd() const;
    Vertex &
    vertex(const size_t index);
    const Vertex &
    vertex(const size_t index) const;
    const Vertex &
    vertexFront() const;
    const Vertex &
    vertexBack() const;

    EdgeConstIterator
    edgesBegin() const;
    EdgeConstIterator
    edgesEnd() const;
    Edge &
    edge(const size_t index);
    const Edge &
    edge(const size_t index) const;
    Edge &
    edgeFront();
    const Edge &
    edgeFront() const;
    Edge &
    edgeBack();
    const Edge &
    edgeBack() const;

    void
    reverse();

    size_t
    numVertices() const;

    size_t
    numEdges() const;

    bool
    isCircular() const;

    bool
    empty() const;

    const Voronoi *
    getVoronoi() const;

    bool
    inRange(const ptrdiff_t index) const;
    ptrdiff_t
    forwardDist(const ptrdiff_t i, const ptrdiff_t j) const;
    ptrdiff_t
    wrapIndex(const ptrdiff_t i) const;
    ptrdiff_t
    safeIndex(const ptrdiff_t i) const;

  private:
    void
    push_back(const typename Delaunay::Edge & edge);

    const Voronoi * myVoronoi;
    Vertices myVertices;
    Edges myEdges;
  };

template< typename VD>
  std::ostream &
  operator <<(std::ostream & os, const VoronoiPath< VD> & path);

}
}

#include "spl/analysis/detail/VoronoiPath.h"

#endif /* SPL_WITH_CGAL */
#endif /* VORONOI_PATH_H */
