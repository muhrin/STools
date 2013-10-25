/*
 * AnchorPointArrangement.h
 *
 *  Created on: Oct 15, 2013
 *      Author: Martin Uhrin
 */

#ifndef ANCHOR_POINT_ARRANGEMENT_H
#define ANCHOR_POINT_ARRANGEMENT_H

// INCLUDES ////////////
#include "spl/SSLib.h"

#ifdef SSLIB_USE_CGAL
#include <map>
#include <vector>

#include <boost/optional.hpp>
#include <boost/ptr_container/ptr_vector.hpp>

#include <armadillo>

#include <CGAL/Polygon_2.h>

#include "spl/analysis/VoronoiEdgeTracer.h"

// FORWARD DECLARATIONS ///////

// DEFINITION ///////////////////////

namespace spl {
namespace analysis {

template< typename LabelType>
  class AnchorPointArrangement;

class AnchorPoint
{
  typedef ::std::set< AnchorPoint *> Neighbours;
public:
  typedef Neighbours::const_iterator NeighbourIterator;

  AnchorPoint(const size_t idx, const ::arma::vec2 & pos,
      const double maxDisplacement);

  const ::arma::vec2 &
  getAnchorPos() const;
  const ::arma::vec2 &
  getPos() const;
  void
  setPos(const ::arma::vec2 & newPos);

  NeighbourIterator
  neighboursBegin() const;
  NeighbourIterator
  neighboursEnd() const;

  double
  getMaxDisplacement() const;

  size_t
  idx() const;

  size_t
  numNeighbours() const;

  void
  addNeighbour(AnchorPoint * const neighbour);

  bool
  hasNeighbour(AnchorPoint * const neighbour) const;

  void
  swapNeighbours(AnchorPoint * const old, AnchorPoint * const newNeighbour);
  void
  clearNeighbours();

private:
  const size_t idx_;
  const ::arma::vec2 anchorPos_;
  ::arma::vec2 pos_;
  const double maxDisplacement_;
  Neighbours neighbours_;
};

template< typename LabelType>
  class AnchorPointArrangement
  {
    struct VertexData : public arrangement_data::Vertex< LabelType>
    {
      AnchorPoint * anchor;
    };
    struct HalfedgeData : public arrangement_data::Halfedge< LabelType>
    {
    };
    struct FaceData : public arrangement_data::Face< LabelType>
    {
    };

  public:
    typedef VoronoiEdgeTracer< LabelType, VertexData, HalfedgeData, FaceData> EdgeTracer;
    typedef typename EdgeTracer::Arrangement Arrangement;

  private:
    typedef ::boost::ptr_vector< AnchorPoint> Points;

  public:
    typedef typename Points::iterator PointsIterator;

    AnchorPointArrangement(EdgeTracer & edgeTracer);

    PointsIterator
    beginPoints();
    PointsIterator
    endPoints();
    size_t
    numPoints() const;
    AnchorPoint *
    getPoint(const size_t index);

    ::arma::mat
    getPointPositions() const;
    void
    setPointPositions(const ::arma::mat & pos);

    AnchorPoint *
    getAnchorPoint(const typename Arrangement::Vertex_const_handle & vertex);

    ::boost::optional<double>
    getFaceAnchorArea(const typename Arrangement::Face & face) const;

  private:
    AnchorPoint *
    getNeighbouringAnchor(AnchorPoint * const anchor,
        const typename Arrangement::Vertex_const_handle & neighbourVertex);
    void
    init();
    void
    generateArrangementVertices();
    void
    generateArrangementEdges();

    EdgeTracer & tracer_;
    Points points_;
  };

}
}

#include "spl/analysis/detail/AnchorPointArrangement.h"

#endif /* SSLIB_USE_CGAL */
#endif /* ANCHOR_POINT_ARRANGEMENT_H */
