/*
 * IConvexHullInfoSupplier.h
 *
 *  Created on: Jul 17, 2013
 *      Author: Martin Uhrin
 */

#ifndef I_CONVEX_HULL_INFO_SUPPLIER_H
#define I_CONVEX_HULL_INFO_SUPPLIER_H

// INCLUDES ////////////
#include "SSLib.h"

#ifdef SSLIB_USE_CGAL

#include "analysis/ConvexHull.h"

// DEFINITION ///////////////////////

namespace sstbx
{

// FORWARD DECLARATIONS ///////

namespace analysis
{

class IConvexHullInfoSupplier
{
public:
  virtual
  ~IConvexHullInfoSupplier()
  {
  }

  virtual ::std::string
  getLabel(const ConvexHull & convexHull,
      const ConvexHull::PointId pointId) const = 0;
};

}
}

#endif // SSLIB_USE_CGAL
#endif /* I_CONVEX_HULL_INFO_SUPPLIER_H */
