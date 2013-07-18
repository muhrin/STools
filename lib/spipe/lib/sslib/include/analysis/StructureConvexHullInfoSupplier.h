/*
 * StructureConvexHullInfoSupplier.h
 *
 *  Created on: Jul 17, 2013
 *      Author: Martin Uhrin
 */

#ifndef STRUCTURE_CONVEX_HULL_INFO_SUPPLIER_H
#define STRUCTURE_CONVEX_HULL_INFO_SUPPLIER_H

// INCLUDES ////////////
#include "SSLib.h"

#ifdef SSLIB_USE_CGAL

#include <map>

#include "analysis/ConvexHull.h"
#include "analysis/IConvexHullInfoSupplier.h"

// DEFINITION ///////////////////////

namespace sstbx {

// FORWARD DECLARATIONS ///////
namespace common {
class Structure;
}

namespace analysis {


class StructureConvexHullInfoSupplier : public IConvexHullInfoSupplier
{
public:

  void addStructure(const common::Structure & structure, const ConvexHull::PointId id);
  virtual ::std::string getLabel(const ConvexHull & convexHull, const ConvexHull::PointId pointId) const;

private:
  typedef ::std::map<ConvexHull::PointId, const common::Structure *> Structures;

  Structures myStructures;
};

}
}

#endif // SSLIB_USE_CGAL

#endif /* STRUCTURE_CONVEX_HULL_INFO_SUPPLIER_H */
