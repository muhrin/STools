/*
 * StructureConvexHullInfoSupplier.cpp
 *
 *  Created on: Jul 17, 2013
 *      Author: Martin Uhrin
 */

#include "analysis/StructureConvexHullInfoSupplier.h"

#ifdef SSLIB_USE_CGAL

#include "common/Structure.h"

namespace sstbx {
namespace analysis {

void StructureConvexHullInfoSupplier::addStructure(const common::Structure & structure, int id)
{
  myStructures[id] = &structure;
}

::std::string StructureConvexHullInfoSupplier::getLabel(const ConvexHull & convexHull, const ConvexHull::PointId pointId) const
{
  ::std::string label;
  Structures::const_iterator it = myStructures.find(pointId);
  if(it == myStructures.end())
    return label;

  return it->second->getName();
}

}
}

#endif // SSLIB_USE_CGAL
