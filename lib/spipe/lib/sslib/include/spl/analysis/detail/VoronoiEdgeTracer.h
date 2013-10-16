/*
 * VoronoiEdgeTracer.h
 *
 *  Created on: Oct 15, 2013
 *      Author: Martin Uhrin
 */

#ifndef VORONOI_EDGE_TRACER_DETAIL_H
#define VORONOI_EDGE_TRACER_DETAIL_H

// INCLUDES ///////////////////
#include <vector>

#include "spl/SSLibAssert.h"

// FORWARD DECLARATIONS ///////

// DEFINITION ///////////////////////

namespace spl {
namespace analysis {

template< typename LabelType>
  VoronoiEdgeTracer< LabelType>::VoronoiEdgeTracer(const Voronoi & voronoi)
  {
    TracingData tracingData(voronoi);
    initArrangement(tracingData);

    tracingData.toVisit = tracingData.boundaryEdges;
    for(typename DelaunayEdgeSet::const_reverse_iterator it =
        tracingData.toVisit.rbegin(); !tracingData.toVisit.empty(); it =
        tracingData.toVisit.rbegin())
    {
      traceEdge(tracingData, --it.base());
    }

    //splitEdges();
  }

template< typename LabelType>
  const typename VoronoiEdgeTracer< LabelType>::Arrangement &
  VoronoiEdgeTracer< LabelType>::getArrangement() const
  {
    return arrangement_;
  }

template< typename LabelType>
  LabelType
  VoronoiEdgeTracer< LabelType>::getInfo(
      const Arrangement::Halfedge_const_handle halfedge) const
  {
    const typename HalfedgeInfo::const_iterator it = halfedgeInfo_.find(
        halfedge.ptr());
    return it->second;
  }

template< typename LabelType>
  void
  VoronoiEdgeTracer< LabelType>::initArrangement(TracingData & tracingData)
  {
    // Fist we need to place all the vertices we're going to use into the
    // unbounded face of the arrangement as this make the subsequent step
    // of creating the edges a lot easier

    typedef ::std::pair< typename VertexMap::iterator, bool> VertexMapInsertReturn;

    Arrangement::Face_handle unboundedFace = arrangement_.unbounded_face();
    const Voronoi & voronoi = tracingData.voronoi;
    const typename Voronoi::Delaunay_graph & delaunay = voronoi.dual();

    for(typename Delaunay::Edge_iterator it = delaunay.edges_begin(), end =
        delaunay.edges_end(); it != end; ++it)
    {
      if(isBoundaryEdge(*it) && !voronoi.edge_rejector()(delaunay, *it))
      {
        tracingData.boundaryEdges.insert(*it);
        const typename Voronoi::Halfedge_handle he = voronoi.dual(*it);

        // Populate the map arrangement with vertices if necessary
        if(he->has_source())
        {
          const VertexMapInsertReturn ret = tracingData.vertexMap.insert(
              ::std::make_pair(he->source(), arrangement_.vertices_end()));

          // Is the vertex new to the arrangement?
          if(ret.second)
            ret.first->second = arrangement_.insert_in_face_interior(
                he->source()->point(), unboundedFace);
        }
        if(he->has_target())
        {
          const VertexMapInsertReturn ret = tracingData.vertexMap.insert(
              ::std::make_pair(he->target(), arrangement_.vertices_end()));

          // Is the vertex new to the arrangement?
          if(ret.second)
            ret.first->second = arrangement_.insert_in_face_interior(
                he->target()->point(), unboundedFace);
        }
      }
    }
  }

template< typename LabelType>
  void
  VoronoiEdgeTracer< LabelType>::traceEdge(TracingData & tracingData,
      typename DelaunayEdgeSet::const_iterator edgeIt)
  {
    SSLIB_ASSERT(isBoundaryEdge(*edgeIt));

    typename Voronoi::Delaunay_edge edge = *edgeIt;
    const typename Voronoi::Halfedge_handle halfEdge = tracingData.voronoi.dual(
        edge);
    if(halfEdge->is_unbounded())
    {
      tracingData.toVisit.erase(edgeIt);
      return;
    }

    const typename VertexMap::const_iterator source =
        tracingData.vertexMap.find(halfEdge->source());
    SSLIB_ASSERT(source != tracingData.vertexMap.end());

    traceEdge(tracingData, halfEdge, source);
  }

template< typename LabelType>
  void
  VoronoiEdgeTracer< LabelType>::traceEdge(TracingData & tracingData,
      const typename Voronoi::Halfedge_handle & halfEdge,
      const typename VertexMap::const_iterator source)
  {
    if(halfEdge->is_unbounded())
      return; // One or both ends are at infinity

    typename Voronoi::Delaunay_edge delaunayEdge = halfEdge->dual();

    if(!isBoundaryEdge(delaunayEdge))
      return; // Not boundary

    const typename DelaunayEdgeSet::const_iterator it =
        tracingData.toVisit.find(delaunayEdge);
    if(it == tracingData.toVisit.end())
      return; // Already visited

    const typename VertexMap::const_iterator target =
        tracingData.vertexMap.find(halfEdge->target());
    SSLIB_ASSERT(target != tracingData.vertexMap.end());

    // Insert the segment corresponding to the edge info the arrangement
    const K::Segment_2 segment(source->second->point(),
        target->second->point());
    const Arrangement::Halfedge_handle heHandle =
        arrangement_.insert_at_vertices(segment, source->second,
            target->second);

    // Save the label either side of the halfedge
    halfedgeInfo_[heHandle.ptr()] = halfEdge->face()->dual()->info();
    halfedgeInfo_[heHandle->twin().ptr()] =
        halfEdge->twin()->face()->dual()->info();

    // Finished this Delaunay edge
    tracingData.toVisit.erase(it);

    // Do the neighbouring edges
    traceEdge(tracingData, halfEdge->next(), target);
    traceEdge(tracingData, halfEdge->opposite()->next(), source);
  }

template< typename LabelType>
  bool
  VoronoiEdgeTracer< LabelType>::isBoundaryEdge(
      const typename Delaunay::Edge & edge) const
  {
    return edge.first->vertex((edge.second + 1) % 3)->info()
        != edge.first->vertex((edge.second + 2) % 3)->info();
  }

template< typename LabelType>
  void
  VoronoiEdgeTracer< LabelType>::splitEdges()
  {
    typedef LabelType Label;
    HalfedgeInfo newInfo;

    ::std::vector<typename Arrangement::Halfedge_handle> toSplit;
    for(typename Arrangement::Edge_iterator it = arrangement_.edges_begin(),
        end = arrangement_.edges_end(); it != end; ++it)
    {
      toSplit.push_back(it);
    }

    typename K::Point_2 p1, p2, midpoint;
    typename Arrangement::Vertex_handle source, mid, target;
    typename Arrangement::Halfedge_const_handle e1, e2;
    Label l1, l2;
    while(!toSplit.empty())
    {
      const typename Arrangement::Halfedge_handle edge = toSplit.back();
      const typename Arrangement::Halfedge_handle twin = edge->twin();

      l1 = halfedgeInfo_[edge.ptr()];
      l2 = halfedgeInfo_[twin.ptr()];

      source = edge->source();
      target = edge->target();

      p1 = source->point();
      p2 = target->point();
      midpoint = ::CGAL::midpoint(p1, p2);

      const typename K::Segment_2 s1(p1, midpoint);
      const typename K::Segment_2 s2(midpoint, p2);

      const typename Arrangement::Face_handle face =
          arrangement_.remove_edge(edge, false, false);
      mid = arrangement_.insert_in_face_interior(midpoint, face);
      e1 = arrangement_.insert_at_vertices(s1, source, mid);
      e2 = arrangement_.insert_at_vertices(s2, mid, target);

      const typename Arrangement::Halfedge_const_handle e1Twin = e1->twin();
      const typename Arrangement::Halfedge_const_handle e2Twin = e2->twin();

      newInfo[e1.ptr()] = l1;
      newInfo[e2.ptr()] = l1;

      newInfo[e1Twin.ptr()] = l2;
      newInfo[e2Twin.ptr()] = l2;

      toSplit.pop_back();
    }

    halfedgeInfo_ = newInfo;
  }

}
}

#endif /* VORONOI_EDGE_TRACER_DETAIL_H */
