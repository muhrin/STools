/*
 * SPipeFactoryYaml.h
 *
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef SPIPE_FACTORY_YAML_H
#define SPIPE_FACTORY_YAML_H

// INCLUDES /////////////////////////////////////////////
#include "StructurePipe.h"

#ifdef SP_USE_YAML

#include <boost/optional.hpp>

#include <pipelib/managed/ManagedBlock.h>

// From SSTbx

// Local includes
#include "StructurePipe.h"
#include "SpTypes.h"
#include "factory/SsLibFactoryYaml.h"

// FORWARD DECLARATIONS ////////////////////////////////////
namespace YAML {
class Node;
}

namespace spipe {
namespace factory {


class SPipeFactoryYaml
{
public:
  typedef ::pipelib::managed::ManagedBlock<StructureDataType, SharedDataType, GlobalDataType>
    ManagedBlockType;
  typedef ::boost::optional<ManagedBlockType> OptionalManagedBlock;


  bool createDetermineSpaceGroupBlock(OptionalManagedBlock & blockOut, const YAML::Node & node) const;
  bool createNiggliReduce(OptionalManagedBlock & blockOut, const YAML::Node & node) const;
  bool createParamPotentialGeomOptimiseBlock(OptionalManagedBlock & blockOut, const YAML::Node & node) const;
  bool createPotentialGeomOptimiseBlock(OptionalManagedBlock & blockOut, const YAML::Node & node) const;
  bool createRandomStructureBlock(OptionalManagedBlock & blockOut, const YAML::Node & node) const;
  bool createRemoveDuplicatesBlock(OptionalManagedBlock & blockOut, const YAML::Node & node) const;
  bool createWriteStructuresBlock(OptionalManagedBlock & blockOut, const YAML::Node & node) const;

//  ::spipe::blocks::RandomStructure *  createBlockRandomCrystal(const YAML::Node & blockNode);
  //::pipelib::DefaultBarrier< ::spipe::StructureDataTyp, ::spipe::SharedDataType> *
  //  createBarrier(const YAML::Node & node);
  //::spipe::blocks::RandomStructure *  createBlockRandomCrystal(const YAML::Node & blockNode);
  //::spipe::blocks::RemoveDuplicates * createDropDuplicates(const YAML::Node & node);
  //::spipe::blocks::NiggliReduction *  createNiggli(const YAML::Node & blockNode);
  //::spipe::blocks::PotentialGo *      createGeomOptimise(const YAML::Node & node);
  //::spipe::blocks::WriteStructure *   createWriteStructures(const YAML::Node & node);

private:

  ::sstbx::factory::SsLibFactoryYaml mySsLibFactory;

};


}
}

#endif // SP_USE_YAML

#endif /* SPIPE_FACTORY_YAML_H */

