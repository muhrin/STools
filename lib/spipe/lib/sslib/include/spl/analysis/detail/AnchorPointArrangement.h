/*
 * AnchorPointArrangement.h
 *
 *  Created on: Oct 15, 2013
 *      Author: Martin Uhrin
 */

#ifndef ANCHOR_POINT_ARRANGEMENT_DETAIL_H
#define ANCHOR_POINT_ARRANGEMENT_DETAIL_H

// INCLUDES ///////////////////
#include <boost/foreach.hpp>
#include <boost/range/iterator_range.hpp>

#include "spl/SSLibAssert.h"

// FORWARD DECLARATIONS ///////

// DEFINITION ///////////////////////

namespace spl {
namespace analysis {

template< typename LabelType>
  AnchorPointArrangement< LabelType>::AnchorPointArrangement(
      EdgeTracer & edgeTracer) :
      tracer_(edgeTracer)
  {
    init();
  }

template< typename LabelType>
  typename AnchorPointArrangement< LabelType>::PointsIterator
  AnchorPointArrangement< LabelType>::beginPoints()
  {
    return points_.begin();
  }

template< typename LabelType>
  typename AnchorPointArrangement< LabelType>::PointsIterator
  AnchorPointArrangement< LabelType>::endPoints()
  {
    return points_.end();
  }

template< typename LabelType>
  size_t
  AnchorPointArrangement< LabelType>::numPoints() const
  {
    return points_.size();
  }

template< typename LabelType>
  AnchorPoint *
  AnchorPointArrangement< LabelType>::getPoint(const size_t index)
  {
    return &points_[index];
  }

template< typename LabelType>
  ::arma::mat
  AnchorPointArrangement< LabelType>::getPointPositions() const
  {
    ::arma::mat pos(2, numPoints());
    for(size_t i = 0; i < numPoints(); ++i)
      pos.col(i) = points_[i].getPos();
    return pos;
  }

template< typename LabelType>
  void
  AnchorPointArrangement< LabelType>::setPointPositions(const ::arma::mat & pos)
  {
    for(size_t i = 0; i < numPoints(); ++i)
      points_[i].setPos(pos.col(i));
  }

template< typename LabelType>
  AnchorPoint *
  AnchorPointArrangement< LabelType>::getAnchorPoint(
      const typename Arrangement::Vertex_const_handle & vertex)
  {
    return vertex->data().anchor;
  }

template< typename LabelType>
::boost::optional<double>
AnchorPointArrangement< LabelType>::getFaceAnchorArea(
    const typename Arrangement::Face & face) const
{
  typedef typename Arrangement::Geometry_traits_2::Kernel Kernel;

  if(face.is_unbounded() || face.is_fictitious())
    return ::boost::optional<double>();

  typename ::CGAL::Polygon_2<Kernel> poly;

  const typename Arrangement::Ccb_halfedge_const_circulator first = face.outer_ccb();
  typename Arrangement::Ccb_halfedge_const_circulator cl = first;
  const AnchorPoint * anchor;
  do
  {
    anchor = cl->source()->data().anchor;
    const ::arma::vec2 & anchorPos = anchor->getPos();
    poly.push_back(typename Kernel::Point_2(anchorPos(0), anchorPos(1)));
    ++cl;
  } while(cl != first);

  return ::std::abs(poly.area());
}

template< typename LabelType>
  AnchorPoint *
  AnchorPointArrangement< LabelType>::getNeighbouringAnchor(
      AnchorPoint * const anchor,
      const typename Arrangement::Vertex_const_handle & neighbourVertex)
  {
    BOOST_FOREACH(AnchorPoint * const neighbour, neighbourVertex->data().anchors)
    {
      if(neighbour->hasNeighbour(anchor))
        return neighbour;
    }
    return NULL;
  }

template< typename LabelType>
  void
  AnchorPointArrangement< LabelType>::init()
  {
    using ::std::pair;
    using ::std::make_pair;

    generateArrangementVertices();
    generateArrangementEdges();
  }

template< typename LabelType>
  void
  AnchorPointArrangement< LabelType>::generateArrangementVertices()
  {
    Arrangement & arrangement = tracer_.getArrangement();

    ::arma::vec2 pt;
    for(typename Arrangement::Vertex_iterator it = arrangement.vertices_begin(),
        end = arrangement.vertices_end(); it != end; ++it)
    {
      pt(0) = it->point()[0];
      pt(1) = it->point()[1];

      points_.push_back(new AnchorPoint(points_.size(), pt, it->data().maxDisplacement));
      it->data().anchor = &points_.back();
    }
  }

template< typename LabelType>
  void
  AnchorPointArrangement< LabelType>::generateArrangementEdges()
  {
    Arrangement & arr = tracer_.getArrangement();
    BOOST_FOREACH(typename Arrangement::Vertex & vertex,
        ::boost::make_iterator_range(arr.vertices_begin(), arr.vertices_end()))
    {
      AnchorPoint * const anchor = vertex.data().anchor;

      if(anchor->numNeighbours() == vertex.degree())
        continue;

      typename Arrangement::Halfedge_around_vertex_const_circulator cl =
          vertex.incident_halfedges();
      const typename Arrangement::Halfedge_around_vertex_const_circulator first =
          cl;
      do
      {
        AnchorPoint * const neighbour = getAnchorPoint(cl->source());
        anchor->addNeighbour(neighbour);
        neighbour->addNeighbour(anchor);
        ++cl;
      }
      while(cl != first);
    }
  }

} // namespace analysis
} // namespace spl

#endif /* ANCHOR_POINT_ARRANGEMENT_DETAIL_H */
