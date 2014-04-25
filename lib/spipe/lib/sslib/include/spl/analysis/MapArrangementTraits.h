/*
 * MapArrangementTraits.h
 *
 *  Created on: Mar 22, 2014
 *      Author: Martin Uhrin
 */

#ifndef MAP_ARRANGEMENT_H
#define MAP_ARRANGEMENT_H

// INCLUDES ////////////
#include "spl/SSLib.h"

#ifdef SPL_WITH_CGAL

#include <CGAL/basic.h>

#ifdef CGAL_USE_CORE

#include <boost/optional.hpp>

#include <CGAL/Cartesian.h>
#include <CGAL/CORE_algebraic_number_traits.h>
#include <CGAL/Arr_Bezier_curve_traits_2.h>
#include <CGAL/Arrangement_2.h>
#include <CGAL/Arr_extended_dcel.h>

// FORWARD DECLARATIONS ///////

// DEFINITION ///////////////////////

namespace spl {
namespace analysis {

template< typename LabelType>
  class MapArrangementTraits
  {
  public:
    typedef LabelType Label;

    typedef CGAL::CORE_algebraic_number_traits NtTraits;
    typedef typename NtTraits::Rational NT;
    typedef typename NtTraits::Rational Rational;
    typedef typename NtTraits::Algebraic Algebraic;
    typedef CGAL::Cartesian< Rational> RatKernel;
    typedef CGAL::Cartesian< Algebraic> AlgKernel;
    typedef RatKernel::Point_2 Point;

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
    typedef CGAL::Arr_Bezier_curve_traits_2< RatKernel, AlgKernel, NtTraits> ArrTraits;
    typedef CGAL::Arr_extended_dcel< ArrTraits, VertexInfo, HalfedgeInfo,
        FaceInfo> Dcel;
    typedef CGAL::Arrangement_2< ArrTraits, Dcel> Arrangement;
  };

}
}

#endif /* SPL_WITH_CGAL */
#endif // CGAL_USE_CORE
#endif /* MAP_ARRANGEMENT_H */
