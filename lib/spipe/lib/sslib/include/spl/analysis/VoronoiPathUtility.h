/*
 * VoronoiPathUtility.h
 *
 *  Created on: Mar 18, 2014
 *      Author: Martin Uhrin
 */

#ifndef VORONOI_PATH_UTILITY_H
#define VORONOI_PATH_UTILITY_H

// INCLUDES ////////////
#include "spl/SSLib.h"

#ifdef SPL_WITH_CGAL

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Polygon_2.h>

#include "spl/analysis/MapArrangement.h"
#include "spl/utility/Range.h"

// FORWARD DECLARATIONS ///////

// DEFINITION ///////////////////////

namespace spl {
namespace analysis {

// Convenience struct-typedef for getting the Delaunay vertex info label from a
// Voronoi diagram type
template< class VD>
  struct VoronoiLabel
  {
    typedef typename VD::Delaunay_graph::Triangulation_data_structure::Vertex::Info Type;
  };

// Convenience struct-typedef for getting the Delaunay vertex info label from a
// Delaunay graph type
template< class DG>
  struct DelaunayLabel
  {
    typedef typename DG::Triangulation_data_structure::Vertex::Info Type;
  };

template< typename Label>
  struct BoundaryPair
  {
    typedef spl::utility::OrderedPair< Label> Type;
  };

template< class VD>
  typename BoundaryPair< typename VoronoiLabel< VD>::Type>::Type
  getBoundaryPair(const typename VD::Halfedge & he);

template< class DG>
  typename BoundaryPair< typename DelaunayLabel< DG>::Type>::Type
  getSpanningPair(const typename DG::Edge & edge);

// Is this Voronoi halfedge a boundary between regions of different labels
template< class VD>
  bool
  isBoundary(const typename VD::Halfedge & he);

// Does this Delaunay edge span the boundary between regions of different
// labels
template< class DG>
  bool
  spansBoundary(const typename DG::Edge & edge);

template< typename VD>
  class VoronoiPathArrangement;

template< typename VD>
  CGAL::Polygon_2< typename VD::Delaunay_geom_traits>
  delaunayDomain(const typename VD::Vertex_handle & vtx, const VD & voronoi);

template< typename Label, typename VD>
  typename MapArrangement< CGAL::Exact_predicates_exact_constructions_kernel,
      Label>::Arrangement
  toMap(const VoronoiPathArrangement< VD> & pathArrangement);

}
}

#include "spl/analysis/detail/VoronoiPathUtility.h"

#endif /* SPL_WITH_CGAL */
#endif /* VORONOI_PATH_UTILITY_H */
