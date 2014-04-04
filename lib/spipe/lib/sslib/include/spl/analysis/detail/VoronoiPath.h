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
    bool
    isBoundary() const
    {
      SSLIB_ASSERT(isValid());

      return myIsBoundary;
    }

    const boost::optional< Line> &
    getLeastSqLine() const
    {
      return myLeastSqLine;
    }
    void
    setLeastSqLine(const boost::optional< Line> & lsq)
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
        myPath(NULL), mySource(0), myTarget(0), myIsBoundary(false)
    {
    }
    Edge(const Path & path, const size_t source, const size_t target,
        const typename Delaunay::Edge & dgEdge, const bool isBoundary) :
        myPath(&path), mySource(source), myTarget(target), myIsBoundary(
            isBoundary), myDelaunayEdge(dgEdge)
    {
      initLabels();
    }
    Edge(const Path & path, const size_t source, const size_t target,
        const typename Delaunay::Edge & delaunayEdge) :
        myPath(&path), mySource(source), myTarget(target), myIsBoundary(false), myDelaunayEdge(
            delaunayEdge)
    {
      // Because this edge has a Delaunay edge that it intersects it is not
      // a boundary edge (as set in the initialiser list)
      initLabels();
    }

    void
    setSource(const size_t source)
    {
      mySource = source;
    }
    void
    setTarget(const size_t target)
    {
      myTarget = target;
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

      // Swap source and target and wrap around
      setTarget(n - source() - 1);
      setSource(n - target() - 1);
      setDelaunayEdge(myPath->getVoronoi()->dual().mirror_edge(delaunayEdge()));
      std::swap(myLeftLabel, myRightLabel);
    }
    void
    initLabels()
    {
      SSLIB_ASSERT(myPath);
      SSLIB_ASSERT(myDelaunayEdge != typename Delaunay::Edge());

      if(myIsBoundary)
      {

      }
      else
      {
        // TODO: Check that this is the right way around
        myLeftLabel = myDelaunayEdge.first->vertex(
            (myDelaunayEdge.second + 1) % 3)->info();
        myRightLabel = myDelaunayEdge.first->vertex(
            (myDelaunayEdge.second + 2) % 3)->info();
      }
    }

    const Path * myPath;
    size_t mySource;
    size_t myTarget;
    bool myIsBoundary;
    typename Delaunay::Edge myDelaunayEdge;
    boost::optional< Line> myLeastSqLine;
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
  size_t
  VoronoiPath< VD>::push_back(const typename Voronoi::Halfedge_handle & he)
  {
    SSLIB_ASSERT(myVoronoi);
    SSLIB_ASSERT(!isCircular());

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
      const typename Delaunay::Edge edge = he->dual();
      const CGAL::Segment_2< GeomTraits> seg = myVoronoi->dual().segment(edge);

      Polygon domain;
      domain.push_back(seg.source());
      domain.push_back(seg.target());

      myVertices.push_back(
          Vertex(CGAL::midpoint(seg.source(), seg.target()), domain, edge));
    }

    // TODO: Check for circular paths
    // Continuing the path
    myEdges.push_back(
        Edge(*this, numVertices() - 2, numVertices() - 1, he->dual()));

    return myEdges.size() - 1;
  }

template< typename VD>
  size_t
  VoronoiPath< VD>::push_back(const Vertex & vtx1, const Vertex & vtx2,
      const typename Delaunay::Edge & dgEdge, const bool isBoundary)
  {
    SSLIB_ASSERT(myVoronoi);
    SSLIB_ASSERT(!isCircular());

    // Create the first source vertex
    if(myVertices.empty())
      myVertices.push_back(vtx1);
    else
      SSLIB_ASSERT(myVertices.back().voronoiVertex() == vtx1.voronoiVertex());

    if(!vtx2.isBoundary()
        && vtx2.voronoiVertex() == myVertices.front().voronoiVertex())
    {
      // The path is circular and this is the last edge
      myEdges.push_back(Edge(*this, numVertices() - 1, 0, dgEdge, isBoundary));
    }
    else
    {
      SSLIB_ASSERT(myVertices.back().point() != vtx2.point());

      myVertices.push_back(vtx2);

      // Continuing the path
      myEdges.push_back(
          Edge(*this, numVertices() - 2, numVertices() - 1, dgEdge,
              isBoundary));
    }

    return myEdges.size() - 1;
  }

template< typename VD>
  void
  VoronoiPath< VD>::push_back(const typename Delaunay::Edge & edge)
  {
    const Delaunay & delaunay = myVoronoi->dual();
    const CGAL::Segment_2< GeomTraits> seg = delaunay.segment(edge);

    const Point & point = CGAL::midpoint(seg.source(), seg.target());

    Polygon domain;
    domain.push_back(seg.source());
    domain.push_back(seg.target());

    myVertices.push_back(Vertex(point, domain, edge));

    if(numVertices() > 1)
    {
      // Check if this arrangement edge spans the supplied Delaunay edge
      const typename Voronoi::Vertex_handle lastVoronoiVtx = vertex(
          numVertices() - 2).voronoiVertex();
      if(myVoronoi->dual(edge)->source() == lastVoronoiVtx)
        myEdges.push_back(
            Edge(*this, numVertices() - 2, numVertices() - 1, edge));
      else
        myEdges.push_back(
            Edge(*this, numVertices() - 2, numVertices() - 1, edge, true));

      // Check if this vertex completes a circular path
      if(myVertices.front().isBoundary()
          && myVertices.front().getBoundaryEdge() == edge)
        myEdges.push_back(Edge(*this, numVertices() - 1, 0, edge, true));
    }
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
  VoronoiPath< VD>::isCircular() const
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
