/*
 * StructureBuilder.h
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef STRUCTURE_BUILDER_H
#define STRUCTURE_BUILDER_H

// INCLUDES ////////////
#include <boost/shared_ptr.hpp>

#include "build_cell/ConstStructureDescriptionVisitor.h"
#include "common/Types.h"

namespace sstbx {
namespace build_cell {

class StructureDescription;
class StructureDescriptionMap;

class StructureBuilder : public ConstStructureDescriptionVisitor
{
public:

  typedef ::boost::shared_ptr<StructureDescriptionMap> DescriptionMapPtr;
  typedef ::std::pair<common::StructurePtr, DescriptionMapPtr> StructurePair;

  StructurePair buildStructure(const StructureDescription & description);

  // From StructureDescriptionVisitor ///////////////////
  virtual bool visitAtom(const AtomsDescription & description);
  // End from StructureDescriptionVisitor ///////////////

  double getAtomsVolume() const;

private:

  StructurePair *   myCurrentPair;
  double            myAtomsVolume;
  
};

}
}

#endif /* STRUCTURE_BUILDER_H */
