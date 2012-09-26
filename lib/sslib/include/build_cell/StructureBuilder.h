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

#include "SSLib.h"
#include "build_cell/ConstStructureDescriptionVisitor.h"
#include "common/Types.h"

namespace sstbx {
namespace build_cell {

class StructureDescription;
class StructureDescriptionMap;

class StructureBuilder : public ConstStructureDescriptionVisitor
{
public:

  typedef UniquePtr<StructureDescriptionMap>::Type DescriptionMapPtr;

  common::StructurePtr buildStructure(const StructureDescription & description, DescriptionMapPtr & outDescriptionMap);

  // From StructureDescriptionVisitor ///////////////////
  virtual bool visitAtom(const AtomsDescription & description);
  // End from StructureDescriptionVisitor ///////////////

  double getAtomsVolume() const;

private:

  typedef ::std::pair<common::Structure *, StructureDescriptionMap *> StructurePair;

  StructurePair     myCurrentPair;
  double            myAtomsVolume;
  
};

}
}

#endif /* STRUCTURE_BUILDER_H */
