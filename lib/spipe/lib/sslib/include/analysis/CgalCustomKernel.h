/*
 * CgalPointDWithId.h
 *
 *  Created on: Jul 17, 2013
 *      Author: Martin Uhrin
 */

#ifndef CGAL_CUSTOM_KERNEL_H
#define CGAL_CUSTOM_KERNEL_H

// INCLUDES ////////////
#include "SSLib.h"

#ifdef SSLIB_USE_CGAL

#include <CGAL/Cartesian.h>
#include <CGAL/Cartesian_d.h>
#include <CGAL/Kernel/Type_equality_wrapper.h>

#include "analysis/CgalPointDWithId.h"

// DEFINITION ///////////////////////

namespace sstbx {

// FORWARD DECLARATIONS ///////

namespace analysis {
namespace detail {

template <typename Kernel, typename KernelBase>
class CgalCustomKernelBase : public KernelBase
{
public:
  typedef typename KernelBase::RT RT;
  typedef typename KernelBase::FT FT;
  typedef typename KernelBase::LA LA;

  typedef CgalPointDWithId<Kernel> Point_d;
  typedef CGAL::Vector_d<Kernel> Vector_d;

  template <typename Kernel2>
  struct Base { typedef CgalCustomKernelBase<Kernel2, KernelBase> Type; };

  struct Vector_to_point_d {
    Point_d operator()(const Vector_d & v) const
    { return analysis::operator +(CGAL::ORIGIN, v); }
  };

  Vector_to_point_d vector_to_point_d_object() const
  { return Vector_to_point_d(); }

};

} // namespace detail


template <typename FT>
struct CgalCustomKernel : public CGAL::Type_equality_wrapper<
  detail::CgalCustomKernelBase<CgalCustomKernel<FT>, CGAL::Cartesian_d<FT> >,
  CgalCustomKernel<FT> >
{};

}
}


#endif // SSLIB_USE_CGAL
#endif /* CGAL_CUSTOM_KERNEL_H */
