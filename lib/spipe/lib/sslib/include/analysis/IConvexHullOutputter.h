/*
 * IConvexHullOutputter.h
 *
 *  Created on: Jul 17, 2013
 *      Author: Martin Uhrin
 */

#ifndef I_CONVEX_HULL_OUTPUTTER_H
#define I_CONVEX_HULL_OUTPUTTER_H

// INCLUDES ////////////
#include "SSLib.h"

#ifdef SSLIB_USE_CGAL

// DEFINITION ///////////////////////

namespace sstbx
{

// FORWARD DECLARATIONS ///////

namespace analysis
{
class ConvexHull;
class IConvexHullInfoSupplier;

class IConvexHullOutputter
{
public:
  virtual
  ~IConvexHullOutputter()
  {
  }

  virtual bool outputHull(const ConvexHull & convexHull) const = 0;
  virtual bool outputHull(const ConvexHull & convexHull, const IConvexHullInfoSupplier * const infoSupplier) const = 0;
};

}
}

#endif // SSLIB_USE_CGAL
#endif /* I_CONVEX_HULL_OUTPUTTER_H */
