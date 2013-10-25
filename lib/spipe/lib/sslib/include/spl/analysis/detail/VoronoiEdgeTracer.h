/*
 * VoronoiEdgeTracer.h
 *
 *  Created on: Oct 15, 2013
 *      Author: Martin Uhrin
 */

#ifndef VORONOI_EDGE_TRACER_DETAIL_H
#define VORONOI_EDGE_TRACER_DETAIL_H

// INCLUDES ///////////////////
#include <map>
#include <vector>

#include <armadillo>

#include <boost/foreach.hpp>
#include <boost/range/iterator_range.hpp>

#include "spl/SSLibAssert.h"

// FORWARD DECLARATIONS ///////

// DEFINITION ///////////////////////

namespace spl {
namespace analysis {

template< typename LabelType, class VertexDataType, class HalfedgeDataType,
    class FaceDataType>
  VoronoiEdgeTracer< LabelType, VertexDataType, HalfedgeDataType, FaceDataType>::VoronoiEdgeTracer(
      const Voronoi & voronoi, const bool _splitSharedVertices)
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

    if(_splitSharedVertices)
      splitSharedVertices();
    //splitEdges();
  }

template< typename LabelType, class VertexDataType, class HalfedgeDataType,
    class FaceDataType>
  typename VoronoiEdgeTracer< LabelType, VertexDataType, HalfedgeDataType,
      FaceDataType>::Arrangement &
  VoronoiEdgeTracer< LabelType, VertexDataType, HalfedgeDataType, FaceDataType>::getArrangement()
  {
    return arrangement_;
  }

template< typename LabelType, class VertexDataType, class HalfedgeDataType,
    class FaceDataType>
  const typename VoronoiEdgeTracer< LabelType, VertexDataType, HalfedgeDataType,
      FaceDataType>::Arrangement &
  VoronoiEdgeTracer< LabelType, VertexDataType, HalfedgeDataType, FaceDataType>::getArrangement() const
  {
    return arrangement_;
  }

template< typename LabelType, class VertexDataType, class HalfedgeDataType,
    class FaceDataType>
  void
  VoronoiEdgeTracer< LabelType, VertexDataType, HalfedgeDataType, FaceDataType>::initArrangement(
      TracingData & tracingData)
  {
    // Fist we need to place all the vertices we're going to use into the
    // unbounded face of the arrangement as this make the subsequent step
    // of creating the edges a lot easier

    typedef ::std::pair< typename VertexMap::iterator, bool> VertexMapInsertReturn;

    typename Arrangement::Face_handle unboundedFace =
        arrangement_.unbounded_face();
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
          {
            ret.first->second = arrangement_.insert_in_face_interior(
                he->source()->point(), unboundedFace);
            ret.first->second->data().maxDisplacement = ::std::sqrt(
                ::CGAL::squared_distance(ret.first->second->point(),
                    it->first->vertex((it->second + 1) % 3)->point()));
          }
        }
        if(he->has_target())
        {
          const VertexMapInsertReturn ret = tracingData.vertexMap.insert(
              ::std::make_pair(he->target(), arrangement_.vertices_end()));

          // Is the vertex new to the arrangement?
          if(ret.second)
          {
            ret.first->second = arrangement_.insert_in_face_interior(
                he->target()->point(), unboundedFace);
            ret.first->second->data().maxDisplacement = ::std::sqrt(
                ::CGAL::squared_distance(ret.first->second->point(),
                    it->first->vertex((it->second + 1) % 3)->point()));
          }
        }
      }
    }
  }

template< typename LabelType, class VertexDataType, class HalfedgeDataType,
    class FaceDataType>
  void
  VoronoiEdgeTracer< LabelType, VertexDataType, HalfedgeDataType, FaceDataType>::traceEdge(
      TracingData & tracingData,
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

template< typename LabelType, class VertexDataType, class HalfedgeDataType,
    class FaceDataType>
  void
  VoronoiEdgeTracer< LabelType, VertexDataType, HalfedgeDataType, FaceDataType>::traceEdge(
      TracingData & tracingData,
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
    const ArrSegment segment(source->second->point(), target->second->point());
    const typename Arrangement::Halfedge_handle heHandle =
        arrangement_.insert_at_vertices(segment, source->second,
            target->second);

    // Save the label either side of the halfedge
    heHandle->data().label = halfEdge->face()->dual()->info();
    heHandle->twin()->data().label = halfEdge->twin()->face()->dual()->info();

    // Finished this Delaunay edge
    tracingData.toVisit.erase(it);

    // Do the neighbouring edges
    traceEdge(tracingData, halfEdge->next(), target);
    traceEdge(tracingData, halfEdge->opposite()->next(), source);
  }

template< typename LabelType, class VertexDataType, class HalfedgeDataType,
    class FaceDataType>
  bool
  VoronoiEdgeTracer< LabelType, VertexDataType, HalfedgeDataType, FaceDataType>::isBoundaryEdge(
      const typename Delaunay::Edge & edge) const
  {
    return edge.first->vertex((edge.second + 1) % 3)->info()
        != edge.first->vertex((edge.second + 2) % 3)->info();
  }

template< typename LabelType, class VertexDataType, class HalfedgeDataType,
    class FaceDataType>
  void
  VoronoiEdgeTracer< LabelType, VertexDataType, HalfedgeDataType, FaceDataType>::splitSharedVertices()
  {
    typedef ::std::vector<
        typename Arrangement::Halfedge_around_vertex_circulator> SurroundingZones;

    using ::std::make_pair;

    typename Arrangement::Vertex_iterator vNext;
    for(typename Arrangement::Vertex_iterator vIt =
        arrangement_.vertices_begin(), vEnd = arrangement_.vertices_end();
        vIt != vEnd; vIt = vNext)
    {
      // Store an iterator to the next vertex (as we may delete vIt and
      // invalidate the iterator).
      vNext = vIt;
      ++vNext;

      typename Arrangement::Vertex & vertex = *vIt;

      const size_t degree = vertex.degree();

      // Can't split vertices with less than 4 zones
      if(degree > 3)
      {
        const typename Arrangement::Halfedge_around_vertex_circulator first =
            vertex.incident_halfedges();
        typename Arrangement::Halfedge_around_vertex_circulator cl = first;

        // Put all the halfedges that correspond to zones into an array
        // so we can lookup using the index
        SurroundingZones zones;
        do
        {
          zones.push_back(cl);
          ++cl;
        }
        while(cl != first);

        VertexSplitter splitter(degree);
        {
          // Find the graph of connected zones
          ::std::vector< bool> visitedZones(degree, false);
          {
            size_t lastZone, numConnections;
            for(size_t i = 0; i < zones.size(); ++i)
            {
              if(!visitedZones[i])
              {
                lastZone = i;
                numConnections = 0;

                // Check counterclockwise for connections
                for(size_t j = i + 1; j < zones.size(); ++j)
                {
                  if(zones[i]->data().label == zones[j]->data().label)
                  {
                    splitter.addEdge(lastZone, j, zones[i]->data().label);
                    lastZone = j;
                    visitedZones[j] = true;
                    ++numConnections;
                  }
                }

                // Check clockwise for a wrap-around connection,
                // e.g. 1->3, 3->5, 5->7 and 7->1 (but expressed as 1->7)
                if(numConnections > 1)
                  splitter.addEdge(i, lastZone, zones[i]->data().label);

                visitedZones[i] = true;
              } // (!visitedZones[i])
            }
          }
        }

        // Get rid of colliding edges
        splitter.resolveCollisions();
        if(splitter.noEdges())
          continue;

        typedef ::std::vector< SplitVertex> NewVertices;

        ::std::vector< int> edgeVertexIndices(degree, -1);
        NewVertices vertices;

        {
          ::std::pair< int, int> zone;
          BOOST_FOREACH(typename VertexSplitter::EdgeIterator::reference entry,
              ::boost::make_iterator_range(splitter.edgesBegin(), splitter.edgesEnd()))
          {
            const typename VertexSplitter::Edge & edge = entry.first;
            {
              SplitVertex sv;

              zone.first = edge.lower();
              zone.second = (edge.upper() + 1) % degree;

              if(edgeVertexIndices[zone.first] == -1)
              {
                sv.addHalfedge(zones[zone.first]);
                edgeVertexIndices[zone.first] = vertices.size();
              }

              if(edgeVertexIndices[zone.second] == -1)
              {
                sv.addHalfedge(zones[zone.second]);
                edgeVertexIndices[zone.second] = vertices.size();
              }

              if(sv.numNeighbours() != 0)
                vertices.push_back(sv);
            }

            {
              SplitVertex sv;

              // Move on to the next zone
              zone.first = (edge.lower() + 1) % degree;
              zone.second = edge.upper();

              if(edgeVertexIndices[zone.first] == -1)
              {
                sv.addHalfedge(zones[zone.first]);
                edgeVertexIndices[zone.first] = vertices.size();
              }

              if(edgeVertexIndices[zone.second] == -1)
              {
                sv.addHalfedge(zones[zone.second]);
                edgeVertexIndices[zone.second] = vertices.size();
              }

              if(sv.numNeighbours() != 0)
                vertices.push_back(sv);
            }
          }
        }

        // Now connect up any vertices that should be at a vertex but are between zones
        {
          int zone = -1;
          // Move to the first connected edge vertex
          for(size_t k = 0; k < degree; ++k)
          {
            if(edgeVertexIndices[k] != -1
                && edgeVertexIndices[(k + 1) % degree] == -1)
            {
              zone = k;
              break;
            }
          }
          if(zone != -1)
          {
            size_t lastVertexIndex = edgeVertexIndices[zone];
            for(size_t k = 0; k < degree; ++k)
            {
              zone = (zone + 1) % degree;
              if(edgeVertexIndices[zone] == -1)
              {
                vertices[lastVertexIndex].addHalfedge(zones[zone]);
                edgeVertexIndices[zone] = lastVertexIndex;
              }
              else
                lastVertexIndex = edgeVertexIndices[zone];
            }
          }
        }

        // First remove all the old halfedges from the arrangement
        BOOST_FOREACH(typename Arrangement::Halfedge_around_vertex_circulator oldHalfedge, zones)
          arrangement_.remove_edge(oldHalfedge, false, false);

        const VertexDataType vertexData = vIt->data();
        const K::Point_2 oldPos = vIt->point();

        // Remove the old vertex
        arrangement_.remove_isolated_vertex(vIt);

        // WARNING: Don't use old vertex (vIt) or old halfedges (zones) beyond this point

        bool doneFirst = false;
        BOOST_FOREACH(const SplitVertex & splitVertex, vertices)
        {
          const K::Point_2 newVertexPos = oldPos
              + 0.1 * vertexData.maxDisplacement * splitVertex.meanPos();

          // Now, create the new 'split' vertex
          const typename Arrangement::Vertex_handle newArrVertex =
              ::CGAL::insert_point(arrangement_, newVertexPos);
          newArrVertex->set_data(vertexData); // Copy the data over

          // Create the new halfedges
          BOOST_FOREACH(typename Arrangement::Vertex_handle neighbour,
              ::boost::make_iterator_range(splitVertex.neighboursBegin(), splitVertex.neighboursEnd()))
          {
            // Insert the segment corresponding to the edge into the arrangement
            const ArrSegment segment(newArrVertex->point(), neighbour->point());
            const typename Arrangement::Halfedge_handle heHandle =
                arrangement_.insert_at_vertices(segment, newArrVertex,
                    neighbour);
          }

        }
      }
    }
  }

template< typename LabelType, class VertexDataType, class HalfedgeDataType,
    class FaceDataType>
  void
  VoronoiEdgeTracer< LabelType, VertexDataType, HalfedgeDataType, FaceDataType>::splitEdges()
  {
    typedef LabelType Label;

    ::std::vector< typename Arrangement::Halfedge_handle> toSplit;
    for(typename Arrangement::Edge_iterator it = arrangement_.edges_begin(),
        end = arrangement_.edges_end(); it != end; ++it)
    {
      toSplit.push_back(it);
    }

    typename K::Point_2 p1, p2, midpoint;
    typename Arrangement::Vertex_handle source, mid, target;
    typename Arrangement::Halfedge_handle e1, e2;
    HalfedgeDataType e1Data, e2Data;
    while(!toSplit.empty())
    {
      const typename Arrangement::Halfedge_handle edge = toSplit.back();
      const typename Arrangement::Halfedge_handle twin = edge->twin();

      e1Data = edge->data();
      e2Data = twin->data();

      source = edge->source();
      source->data().maxDisplacement *= 0.5;
      target = edge->target();
      target->data().maxDisplacement *= 0.5;

      p1 = source->point();
      p2 = target->point();
      midpoint = ::CGAL::midpoint(p1, p2);

      const typename K::Segment_2 s1(p1, midpoint);
      const typename K::Segment_2 s2(midpoint, p2);

      const typename Arrangement::Face_handle face = arrangement_.remove_edge(
          edge, false, false);
      mid = arrangement_.insert_in_face_interior(midpoint, face);
      mid->set_data(source->data());
      e1 = arrangement_.insert_at_vertices(s1, source, mid);
      e2 = arrangement_.insert_at_vertices(s2, mid, target);

      const typename Arrangement::Halfedge_handle e1Twin = e1->twin();
      const typename Arrangement::Halfedge_handle e2Twin = e2->twin();

      e1->set_data(e1Data);
      e2->set_data(e1Data);

      e1Twin->set_data(e1Data);
      e2Twin->set_data(e2Data);

      toSplit.pop_back();
    }
  }

template< typename LabelType, class VertexDataType, class HalfedgeDataType,
    class FaceDataType>
  VoronoiEdgeTracer< LabelType, VertexDataType, HalfedgeDataType, FaceDataType>::VertexSplitter::VertexSplitter(
      const size_t degree) :
      degree_(degree)
  {
  }

template< typename LabelType, class VertexDataType, class HalfedgeDataType,
    class FaceDataType>
  void
  VoronoiEdgeTracer< LabelType, VertexDataType, HalfedgeDataType, FaceDataType>::VertexSplitter::addEdge(
      const size_t i, const size_t j, const LabelType & label)
  {
    edges_[Edge(i, j)] = label;
  }

template< typename LabelType, class VertexDataType, class HalfedgeDataType,
    class FaceDataType>
  typename VoronoiEdgeTracer< LabelType, VertexDataType, HalfedgeDataType,
      FaceDataType>::VertexSplitter::EdgeIterator
  VoronoiEdgeTracer< LabelType, VertexDataType, HalfedgeDataType, FaceDataType>::VertexSplitter::edgesBegin() const
  {
    return edges_.begin();
  }

template< typename LabelType, class VertexDataType, class HalfedgeDataType,
    class FaceDataType>
  typename VoronoiEdgeTracer< LabelType, VertexDataType, HalfedgeDataType,
      FaceDataType>::VertexSplitter::EdgeIterator
  VoronoiEdgeTracer< LabelType, VertexDataType, HalfedgeDataType, FaceDataType>::VertexSplitter::edgesEnd() const
  {
    return edges_.end();
  }

template< typename LabelType, class VertexDataType, class HalfedgeDataType,
    class FaceDataType>
  bool
  VoronoiEdgeTracer< LabelType, VertexDataType, HalfedgeDataType, FaceDataType>::VertexSplitter::noEdges() const
  {
    return edges_.empty();
  }

template< typename LabelType, class VertexDataType, class HalfedgeDataType,
    class FaceDataType>
  size_t
  VoronoiEdgeTracer< LabelType, VertexDataType, HalfedgeDataType, FaceDataType>::VertexSplitter::numEdges() const
  {
    return edges_.size();
  }

template< typename LabelType, class VertexDataType, class HalfedgeDataType,
    class FaceDataType>
  void
  VoronoiEdgeTracer< LabelType, VertexDataType, HalfedgeDataType, FaceDataType>::VertexSplitter::resolveCollisions()
  {
    using ::std::make_pair;

    ::std::vector<
        ::std::vector< ::std::vector< ::std::pair< Edge, LabelType> > > > collisionMatrix(
        degree_);
    // Create a lower triangular matrix of Edges
    for(int i = 0; i < degree_; ++i)
      collisionMatrix[i].resize(i + 1);

    // Perform edge collision detection
    BOOST_FOREACH(typename Edges::const_reference entry, edges_)
    {
      for(size_t i = entry.first.lower(); i <= entry.first.upper(); ++i)
      {
        // Sweep the edge square
        // bottom left to bottom right
        collisionMatrix[entry.first.upper()][i].push_back(entry);
        // top left to bottom right
        collisionMatrix[i][entry.first.lower()].push_back(entry);
      }
    }

    ::std::set< Edge> toRemove;
    // Now all the matrix entries with two or more edges that don't have a common
    // label will be considered 'colliding' and should be removed
    for(int i = 0; i < degree_; ++i)
    {
      for(int j = 0; j < i; ++j)
      {
        if(collisionMatrix[i][j].size() > 1)
        {
          // Test all pairs of edges for different edge labels
          for(int k = 0; k < collisionMatrix[i][j].size(); ++k)
          {
            for(int l = k + 1; l < collisionMatrix[i][j].size(); ++l)
            {
              const LabelType l1 = collisionMatrix[i][j][k].second;
              const LabelType l2 = collisionMatrix[i][j][l].second;
              if(l1 != l2)
              {
                toRemove.insert(collisionMatrix[i][j][k].first);
                toRemove.insert(collisionMatrix[i][j][l].first);
              }
            }
          }
        }
      }
    }

    BOOST_FOREACH(const Edge & edge, toRemove)
      edges_.erase(edge);
  }

template< typename LabelType, class VertexDataType, class HalfedgeDataType,
    class FaceDataType>
  VoronoiEdgeTracer< LabelType, VertexDataType, HalfedgeDataType, FaceDataType>::SplitVertex::SplitVertex() :
      newPos_(::CGAL::NULL_VECTOR)
  {
  }

template< typename LabelType, class VertexDataType, class HalfedgeDataType,
    class FaceDataType>
  void
  VoronoiEdgeTracer< LabelType, VertexDataType, HalfedgeDataType, FaceDataType>::SplitVertex::addHalfedge(
      const typename Arrangement::Halfedge_around_vertex_circulator & halfedge)
  {
    halfedges_.push_back(halfedge);
    neighbours_.push_back(halfedge->source());
    const K::Vector_2 dr = halfedge->source()->point()
        - halfedge->target()->point();
    newPos_ = newPos_ + dr / ::CGAL::sqrt(dr.squared_length());
  }

template< typename LabelType, class VertexDataType, class HalfedgeDataType,
    class FaceDataType>
  size_t
  VoronoiEdgeTracer< LabelType, VertexDataType, HalfedgeDataType, FaceDataType>::SplitVertex::numNeighbours() const
  {
    return neighbours_.size();
  }

template< typename LabelType, class VertexDataType, class HalfedgeDataType,
    class FaceDataType>
  typename VoronoiEdgeTracer< LabelType, VertexDataType, HalfedgeDataType,
      FaceDataType>::SplitVertex::NeighbourIterator
  VoronoiEdgeTracer< LabelType, VertexDataType, HalfedgeDataType, FaceDataType>::SplitVertex::neighboursBegin() const
  {
    return neighbours_.begin();
  }

template< typename LabelType, class VertexDataType, class HalfedgeDataType,
    class FaceDataType>
  typename VoronoiEdgeTracer< LabelType, VertexDataType, HalfedgeDataType,
      FaceDataType>::SplitVertex::NeighbourIterator
  VoronoiEdgeTracer< LabelType, VertexDataType, HalfedgeDataType, FaceDataType>::SplitVertex::neighboursEnd() const
  {
    return neighbours_.end();
  }

template< typename LabelType, class VertexDataType, class HalfedgeDataType,
    class FaceDataType>
  typename VoronoiEdgeTracer< LabelType, VertexDataType, HalfedgeDataType,
      FaceDataType>::SplitVertex::HalfedgeIterator
  VoronoiEdgeTracer< LabelType, VertexDataType, HalfedgeDataType, FaceDataType>::SplitVertex::halfedgesBegin() const
  {
    return halfedges_.begin();
  }

template< typename LabelType, class VertexDataType, class HalfedgeDataType,
    class FaceDataType>
  typename VoronoiEdgeTracer< LabelType, VertexDataType, HalfedgeDataType,
      FaceDataType>::SplitVertex::HalfedgeIterator
  VoronoiEdgeTracer< LabelType, VertexDataType, HalfedgeDataType, FaceDataType>::SplitVertex::halfedgesEnd() const
  {
    return halfedges_.end();
  }

template< typename LabelType, class VertexDataType, class HalfedgeDataType,
    class FaceDataType>
  typename VoronoiEdgeTracer< LabelType, VertexDataType, HalfedgeDataType,
      FaceDataType>::K::Vector_2
  VoronoiEdgeTracer< LabelType, VertexDataType, HalfedgeDataType, FaceDataType>::SplitVertex::meanPos() const
  {
    return newPos_ / neighbours_.size();
  }
}
}

#endif /* VORONOI_EDGE_TRACER_DETAIL_H */
