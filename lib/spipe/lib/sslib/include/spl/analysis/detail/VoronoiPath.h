/*
 * VoronoiPath.h
 *
 *  Created on: Mar 18, 2014
 *      Author: Martin Uhrin
 */

#ifndef VORONOI_PATH_DETAIL_H
#define VORONOI_PATH_DETAIL_H

// INCLUDES ////////////
#include "spl/SSLib.h"

#ifdef SPL_WITH_CGAL

#include <ostream>

#include <boost/foreach.hpp>

#include <CGAL/Line_2.h>
#include <CGAL/Linear_algebraCd.h>
#include <CGAL/Polygon_2.h>

#include "spl/analysis/VoronoiPathUtility.h"

// FORWARD DECLARATIONS ///////

// DEFINITION ///////////////////////

namespace spl {
namespace analysis {

template< typename VD>
  class VoronoiPath< VD>::Vertex
  {
  public:
    typedef Polygon Domain;

    Vertex()
    {
    }
    Vertex(const Point & point, const Polygon & domain,
        const typename Delaunay::Edge & edge) :
        myPoint(point), myDomain(domain), myBoundaryEdge(edge)
    {
    }
    Vertex(const Point & point, const Polygon & domain,
        const typename Voronoi::Vertex_handle & vtx) :
        myPoint(point), myDomain(domain), myVoronoiVertex(vtx)
    {
    }

    Point &
    point()
    {
      return myPoint;
    }
    const Point &
    point() const
    {
      return myPoint;
    }
    const typename VD::Vertex_handle &
    voronoiVertex() const
    {
      return myVoronoiVertex;
    }
    const Polygon
    domain() const
    {
      return myDomain;
    }

    bool
    isBoundary() const
    {
      return myVoronoiVertex == typename Voronoi::Vertex_handle();
    }

    const typename Delaunay::Edge &
    getBoundaryEdge() const
    {
      return myBoundaryEdge;
    }

  private:
    Point myPoint;
    Polygon myDomain;

    // If the vertex is an internal vertex then it will have the corresponding
    // Voronoi vertex
    typename VD::Vertex_handle myVoronoiVertex;
    // If the vertex is a boundary vertex then it will have the corresponding
    // Delaunay edge that it sits on
    typename Delaunay::Edge myBoundaryEdge;
  };

template< typename VD>
  class VoronoiPath< VD>::Edge
  {
    typedef VoronoiPath< VD> Path;
    typedef CGAL::Linear_algebraCd< typename GeomTraits::FT> Linalg;
  public:
    typedef CGAL::Line_2< GeomTraits> Line;
    typedef typename Linalg::Matrix Matrix;
    typedef std::pair< Line, Point> LineAndCoM;

    size_t
    source() const
    {
      SSLIB_ASSERT(isValid());
      return mySource;
    }
    size_t
    target() const
    {
      SSLIB_ASSERT(isValid());
      return myTarget;
    }
    typename Delaunay::Edge
    delaunayEdge() const
    {
      return myDelaunayEdge;
    }
    bool
    isValid() const
    {
      return myPath;
    }

    const boost::optional< LineAndCoM> &
    getLeastSqLine() const
    {
      return myLeastSqLine;
    }
    void
    setLeastSqLine(const boost::optional< LineAndCoM> & lsq)
    {
      myLeastSqLine = lsq;
    }

    const boost::optional< Matrix> &
    getQuadraticForm() const
    {
      return myQuadraticForm;
    }
    void
    setQuadraticForm(const boost::optional< Matrix> & Q)
    {
      myQuadraticForm = Q;
    }
    boost::optional< Label>
    leftLabel() const
    {
      return myLeftLabel;
    }
    boost::optional< Label>
    rightLabel() const
    {
      return myRightLabel;
    }

  private:
    Edge() :
        myPath(NULL), mySource(0), myTarget(0)
    {
    }
    Edge(const Path & path, const size_t source, const size_t target,
        const typename Delaunay::Edge & delaunayEdge) :
        myPath(&path), mySource(source), myTarget(target), myDelaunayEdge(
            delaunayEdge)
    {
      SSLIB_ASSERT(myDelaunayEdge != typename Delaunay::Edge());
      initLabels();
    }

    void
    setDelaunayEdge(const typename Delaunay::Edge edge)
    {
      myDelaunayEdge = edge;
    }
    void
    reverse()
    {
      SSLIB_ASSERT(isValid());
      const size_t n = myPath->numVertices();

      const size_t newSource = n - target() - 1;
      const size_t newTarget = n - source() - 1;

      // Swap source and target and wrap around
      mySource = newSource;
      myTarget = newTarget;
      setDelaunayEdge(myPath->getVoronoi()->dual().mirror_edge(delaunayEdge()));
      std::swap(myLeftLabel, myRightLabel);
    }
    void
    initLabels()
    {
      SSLIB_ASSERT(isValid());

      const Delaunay & delaunay = myPath->getVoronoi()->dual();

      myLeftLabel = myDelaunayEdge.first->vertex(
          delaunay.ccw(myDelaunayEdge.second))->info();
      myRightLabel = myDelaunayEdge.first->vertex(
          delaunay.cw(myDelaunayEdge.second))->info();
      std::cout << myPath->vertex(mySource).point() << " "
          << myPath->vertex(myTarget).point() - myPath->vertex(mySource).point()
          << " ";
      std::cout << *myLeftLabel << ":" << *myRightLabel << "\n";
    }
    void
    setPath(const Path & path)
    {
      myPath = &path;
    }

    const Path * myPath;
    size_t mySource;
    size_t myTarget;
    typename Delaunay::Edge myDelaunayEdge;
    boost::optional< LineAndCoM> myLeastSqLine;
    boost::optional< Matrix> myQuadraticForm;
    boost::optional< Label> myLeftLabel;
    boost::optional< Label> myRightLabel;

    friend class VoronoiPath< VD> ;
  };

template< typename VD>
  VoronoiPath< VD>::VoronoiPath() :
      myVoronoi(NULL)
  {
  }

template< typename VD>
  VoronoiPath< VD>::VoronoiPath(const Voronoi & voronoi) :
      myVoronoi(&voronoi)
  {
  }

template< typename VD>
  VoronoiPath< VD>::VoronoiPath(const VoronoiPath & path) :
      myVoronoi(path.myVoronoi), myVertices(path.myVertices), myEdges(
          path.myEdges)
  {
    BOOST_FOREACH(Edge & edge, myEdges)
      edge.setPath(*this);
  }

template< typename VD>
  size_t
  VoronoiPath< VD>::push_back(const typename Voronoi::Halfedge_handle & he)
  {
    SSLIB_ASSERT(myVoronoi);
    SSLIB_ASSERT(!isClosed());

    typedef typename Voronoi::Delaunay_graph::Edge DelaunayEdge;

    typename Voronoi::Vertex_handle vtx;
    Polygon domain;

    if(myVertices.empty())
    {
      // Build up the vertex
      if(he->has_source())
      {
        vtx = he->source();
        domain = delaunayDomain(vtx, *myVoronoi);
        myVertices.push_back(
            Vertex(
                CGAL::centroid(domain.vertices_begin(), domain.vertices_end()),
                domain, vtx));
      }
      else
      {
        // We're at the end of the path so use the Delaunay edge as the domain
        // and leave the halfedge empty
        push_back(he->dual());
      }
    }
    else
      // check the source matches the last target
      SSLIB_ASSERT(myVertices.back().voronoiVertex() == he->source());

    // Build up the target vertex
    if(he->has_target())
    {
      vtx = he->target();
      domain = delaunayDomain(vtx, *myVoronoi);
      myVertices.push_back(
          Vertex(CGAL::centroid(domain.vertices_begin(), domain.vertices_end()),
              domain, vtx));
    }
    else
    {
      // We're at the end of the path so use the Delaunay edge as the domain
      // and leave the halfedge empty
      push_back(he->dual());
    }
    // Continue the path
    myEdges.push_back(
        Edge(*this, numVertices() - 2, numVertices() - 1, he->dual()));

    if(numEdges() > 1 && edgeFront().delaunayEdge() != DelaunayEdge())
    {
      // Check that we don't have a closed path
      SSLIB_ASSERT(edgeFront().delaunayEdge() != edgeBack().delaunayEdge());
    }

    return myEdges.size() - 1;
  }

template< typename VD>
  size_t
  VoronoiPath< VD>::push_back(const Vertex & vtx1, const Vertex & vtx2,
      const VoronoiPath & path, const DelaunayEdge & dgEdge)
  {
    SSLIB_ASSERT(myVoronoi);
    SSLIB_ASSERT(!isClosed());

    // Create the first source vertex
    if(myVertices.empty())
      myVertices.push_back(vtx1);
    else
      SSLIB_ASSERT(myVertices.back().voronoiVertex() == vtx1.voronoiVertex());

    if(path.isClosed() && !vtx2.isBoundary()
        && vtx2.voronoiVertex() == vertexFront().voronoiVertex())
    {
      // The path is circular and this is the last edge
      myEdges.push_back(Edge(*this, numVertices() - 1, 0, dgEdge));
    }
    else
    {
      SSLIB_ASSERT(myVertices.back().point() != vtx2.point());

      myVertices.push_back(vtx2);

      // Continuing the path
      myEdges.push_back(
          Edge(*this, numVertices() - 2, numVertices() - 1, dgEdge));
    }

    return myEdges.size() - 1;
  }

template< typename VD>
  size_t
  VoronoiPath< VD>::close(const typename Voronoi::Halfedge_handle & he)
  {
    SSLIB_ASSERT(numVertices() > 1);
    SSLIB_ASSERT(
        he->has_target() && he->target() == vertexFront().voronoiVertex());

    // Complete the circular path
    myEdges.push_back(Edge(*this, numVertices() - 1, 0, he->dual()));
    return myEdges.size() - 1;
  }

template< typename VD>
  typename VoronoiPath< VD>::VertexConstIterator
  VoronoiPath< VD>::verticesBegin() const
  {
    return myVertices.begin();
  }

template< typename VD>
  typename VoronoiPath< VD>::VertexConstIterator
  VoronoiPath< VD>::verticesEnd() const
  {
    return myVertices.end();
  }

template< typename VD>
  typename VoronoiPath< VD>::Vertex &
  VoronoiPath< VD>::vertex(const size_t index)
  {
    return myVertices[index];
  }

template< typename VD>
  const typename VoronoiPath< VD>::Vertex &
  VoronoiPath< VD>::vertex(const size_t index) const
  {
    return myVertices[index];
  }

template< typename VD>
  const typename VoronoiPath< VD>::Vertex &
  VoronoiPath< VD>::vertexFront() const
  {
    return myVertices.front();
  }

template< typename VD>
  const typename VoronoiPath< VD>::Vertex &
  VoronoiPath< VD>::vertexBack() const
  {
    return myVertices.back();
  }

template< typename VD>
  typename VoronoiPath< VD>::EdgeConstIterator
  VoronoiPath< VD>::edgesBegin() const
  {
    return myEdges.begin();
  }

template< typename VD>
  typename VoronoiPath< VD>::EdgeConstIterator
  VoronoiPath< VD>::edgesEnd() const
  {
    return myEdges.end();
  }

template< typename VD>
  typename VoronoiPath< VD>::Edge &
  VoronoiPath< VD>::edge(const size_t index)
  {
    return myEdges[index];
  }

template< typename VD>
  const typename VoronoiPath< VD>::Edge &
  VoronoiPath< VD>::edge(const size_t index) const
  {
    return myEdges[index];
  }

template< typename VD>
  typename VoronoiPath< VD>::Edge &
  VoronoiPath< VD>::edgeFront()
  {
    return myEdges.front();
  }

template< typename VD>
  const typename VoronoiPath< VD>::Edge &
  VoronoiPath< VD>::edgeFront() const
  {
    return myEdges.front();
  }

template< typename VD>
  typename VoronoiPath< VD>::Edge &
  VoronoiPath< VD>::edgeBack()
  {
    return myEdges.back();
  }

template< typename VD>
  const typename VoronoiPath< VD>::Edge &
  VoronoiPath< VD>::edgeBack() const
  {
    return myEdges.back();
  }

template< typename VD>
  void
  VoronoiPath< VD>::reverse()
  {
    std::reverse(myVertices.begin(), myVertices.end());
    std::reverse(myEdges.begin(), myEdges.end());

    BOOST_FOREACH(Edge & edge, myEdges)
      edge.reverse();
  }

template< typename VD>
  size_t
  VoronoiPath< VD>::numVertices() const
  {
    return myVertices.size();
  }

template< typename VD>
  size_t
  VoronoiPath< VD>::numEdges() const
  {
    return myEdges.size();
  }

template< typename VD>
  bool
  VoronoiPath< VD>::isClosed() const
  {
    // If the final edge has the 0th vertex as its target then the path
    // is circular
    return myEdges.size() > 1
        && (myEdges.front().source() == myEdges.back().target());
  }

template< typename VD>
  bool
  VoronoiPath< VD>::empty() const
  {
    return myVertices.empty();
  }

template< typename VD>
  const typename VoronoiPath< VD>::Voronoi *
  VoronoiPath< VD>::getVoronoi() const
  {
    return myVoronoi;
  }

template< typename VD>
  bool
  VoronoiPath< VD>::inRange(const ptrdiff_t i) const
  {
    return i >= 0 && i < numVertices();
  }

template< typename VD>
  ptrdiff_t
  VoronoiPath< VD>::forwardDist(const ptrdiff_t i, const ptrdiff_t j) const
  {
    SSLIB_ASSERT(inRange(i));
    SSLIB_ASSERT(inRange(j));

    const ptrdiff_t n = numVertices();

    if(isClosed())
      return i <= j ? j - i : n - i + j;
    else
    {
      SSLIB_ASSERT(i <= j);
      return j - i;
    }
  }

template< typename VD>
  ptrdiff_t
  VoronoiPath< VD>::wrapIndex(const ptrdiff_t i) const
  {
    if(isClosed())
      return safeIndex(i);
    else
    {
      SSLIB_ASSERT(inRange(i));
      return i;
    }
  }

template< typename VD>
  ptrdiff_t
  VoronoiPath< VD>::safeIndex(const ptrdiff_t i) const
  {
    const ptrdiff_t n = numVertices();
    if(isClosed())
      return i < 0 ? n - (-i % n) : i % n;
    else
      return i < 0 ? 0 : (i >= n ? n - 1 : i);
  }

template< typename VD>
  void
  VoronoiPath< VD>::push_back(const typename Delaunay::Edge & edge)
  {
    const CGAL::Segment_2< GeomTraits> seg = myVoronoi->dual().segment(edge);

    Polygon domain;
    domain.push_back(seg.source());
    domain.push_back(seg.target());

    myVertices.push_back(
        Vertex(CGAL::midpoint(seg.source(), seg.target()), domain, edge));
  }

template< typename VD>
  std::ostream &
  operator <<(std::ostream & os, const VoronoiPath< VD> & path)
  {
    BOOST_FOREACH(const typename VoronoiPath< VD>::Vertex & vtx,
        boost::make_iterator_range(path.verticesBegin(), path.verticesEnd()))
      os << vtx.point() << "\n";
    os << "\n" << std::endl;
    return os;
  }

}
}

#endif /* SPL_WITH_CGAL */
#endif /* VORONOI_PATH_DETAIL_H */
