/*
 * TraceCurve.h
 *
 *  Created on: April 23, 2014
 *      Author: Martin Uhrin
 */

#ifndef TRACE_CURVE_DETAIL_H
#define TRACE_CURVE_DETAIL_H

// INCLUDES ////////////
#include "spl/SSLib.h"

#ifdef SPL_WITH_CGAL

// FORWARD DECLARATIONS ///////

// DEFINITION ///////////////////////

namespace spl {
namespace analysis {

template< typename VD>
  class TraceCurve< VD>::Vertex
  {
  public:
    Vertex()
    {
    }
    explicit
    Vertex(const Point & point) :
        myPoint(point)
    {

    }

    const Point &
    point() const
    {
      return myPoint;
    }
    Point &
    point()
    {
      return myPoint;
    }

  private:
    Point myPoint;
  };

template< typename VD>
  void
  TraceCurve< VD>::pushBack(const Point & point)
  {
    myVertices.push_back(Vertex(point));
  }

template< typename VD>
  size_t
  TraceCurve< VD>::numVertices() const
  {
    return myVertices.size();
  }

template< typename VD>
  const typename TraceCurve< VD>::Vertex &
  TraceCurve< VD>::vertexFront() const
  {
    return myVertices.front();
  }

template< typename VD>
  typename TraceCurve< VD>::Vertex &
  TraceCurve< VD>::vertexFront()
  {
    return myVertices.front();
  }

template< typename VD>
  const typename TraceCurve< VD>::Vertex &
  TraceCurve< VD>::vertexBack() const
  {
    return myVertices.back();
  }

template< typename VD>
  typename TraceCurve< VD>::Vertex &
  TraceCurve< VD>::vertexBack()
  {
    return myVertices.back();
  }

template< typename VD>
  const typename TraceCurve< VD>::Vertex &
  TraceCurve< VD>::vertex(const size_t i) const
  {
    return myVertices[i];
  }

}
}

#endif /* SPL_WITH_CGAL */
#endif /* TRACE_CURVE_DETAIL_H */
