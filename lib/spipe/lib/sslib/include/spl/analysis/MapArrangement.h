/*
 * MapArrangement.h
 *
 *  Created on: Mar 22, 2014
 *      Author: Martin Uhrin
 */

#ifndef MAP_ARRANGEMENT_H
#define MAP_ARRANGEMENT_H

// INCLUDES ////////////
#include "spl/SSLib.h"

#ifdef SPL_WITH_CGAL

#include <boost/optional.hpp>

#include <CGAL/Arrangement_2.h>
#include <CGAL/Arr_extended_dcel.h>
#include <CGAL/Arr_segment_traits_2.h>

// FORWARD DECLARATIONS ///////

// DEFINITION ///////////////////////

namespace spl {
namespace analysis {

template< typename K, typename LabelType>
  class MapArrangement
  {
  public:
    typedef LabelType Label;
    typedef K Kernel;

    struct VertexInfo
    {
    };

    struct HalfedgeInfo
    {
      HalfedgeInfo() :
          label()
      {
      }
      boost::optional< Label> label;
    };

    struct FaceInfo
    {
      boost::optional< Label> label;
    };

    // Arrangements stuff
    typedef CGAL::Arr_segment_traits_2< K> ArrTraits;
    typedef CGAL::Arr_extended_dcel< ArrTraits, VertexInfo, HalfedgeInfo,
        FaceInfo> Dcel;
    typedef CGAL::Arrangement_2< ArrTraits, Dcel> Arrangement;
  };

}
}

#endif /* SPL_WITH_CGAL */
#endif /* MAP_ARRANGEMENT_H */
