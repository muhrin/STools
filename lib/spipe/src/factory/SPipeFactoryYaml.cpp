/*
 * SPipeFactoryYaml.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "factory/SPipeFactoryYaml.h"

#ifdef SP_USE_YAML

#include <yaml-cpp/yaml.h>

// SSLib includes
#include <potential/Types.h>

// Local includes
#include "blocks/DetermineSpaceGroup.h"
#include "blocks/NiggliReduction.h"
#include "blocks/ParamPotentialGo.h"
#include "blocks/RandomStructure.h"
#include "blocks/RemoveDuplicates.h"
#include "blocks/WriteStructure.h"
#include "common/StructureData.h"
#include "common/SharedData.h"
#include "common/YamlKeywords.h"
#include "potential/OptimisationSettings.h"

// NAMESPACES ////////////////////////////////

namespace spipe {
namespace factory {

// Alias for accessing keywords namespace
namespace kw      = ::spipe::common::yaml_keywords;
namespace sslibkw = ::sstbx::factory::sslib_yaml_keywords;
namespace spb     = ::spipe::blocks;
namespace ssbc    = ::sstbx::build_cell;
namespace ssio    = ::sstbx::io;
namespace ssp     = ::sstbx::potential;

bool
SPipeFactoryYaml::createDetermineSpaceGroupBlock(OptionalManagedBlock & blockOut, const YAML::Node & node) const
{
  blockOut.reset(
    ManagedBlockType::create(sstbx::makeUniquePtr(new blocks::DetermineSpaceGroup()))
  );
  return true;
}

bool
SPipeFactoryYaml::createNiggliReduce(OptionalManagedBlock & blockOut, const YAML::Node & node) const
{
  blockOut.reset(
    ManagedBlockType::create(sstbx::makeUniquePtr(new blocks::NiggliReduction()))
  );
  return true;
}

bool
SPipeFactoryYaml::createPotentialGeomOptimiseBlock(OptionalManagedBlock & blockOut, const YAML::Node & node) const
{
  ssp::IGeomOptimiserPtr optimiser;
  if(node[sslibkw::OPTIMISER])
    optimiser = mySsLibFactory.createGeometryOptimiser(node[sslibkw::OPTIMISER]);
  if(!optimiser.get())
    return false;

  ::boost::optional<ssp::OptimisationSettings> settings;
  try
  {
    settings.reset(node.as<ssp::OptimisationSettings>());
  }
  catch(const YAML::TypedBadConversion< ssp::OptimisationSettings> & /*e*/)
  {}

  if(settings)
  {
    blockOut.reset(
      ManagedBlockType::create(sstbx::makeUniquePtr(new blocks::ParamPotentialGo(optimiser, settings)))
    );
  }
  else
  {
    blockOut.reset(
      ManagedBlockType::create(sstbx::makeUniquePtr(new blocks::ParamPotentialGo(optimiser)))
    );
  }

  return true;
}

bool
SPipeFactoryYaml::createRandomStructureBlock(OptionalManagedBlock & blockOut, const YAML::Node & node) const
{
  // Try to construct a structure generator
  ssbc::IStructureGeneratorPtr generator = mySsLibFactory.createStructureGenerator(node);

  unsigned int numToGenerate = 100;
  if(node[kw::BLOCKS__RANDOM_STRUCTURE__NUM])
  {
    try
    {
      numToGenerate = node[kw::BLOCKS__RANDOM_STRUCTURE__NUM].as<unsigned int>();
    }
    catch(const YAML::Exception & /*e*/)
    {}
  }

  blockOut.reset(
    ManagedBlockType::create(sstbx::makeUniquePtr(new blocks::RandomStructure(numToGenerate, generator)))
  );

  return true;
}

bool
SPipeFactoryYaml::createRemoveDuplicatesBlock(OptionalManagedBlock & blockOut, const YAML::Node & node) const
{
  // TODO: Finish
  return false;
}

bool
SPipeFactoryYaml::createWriteStructuresBlock(OptionalManagedBlock & blockOut, const YAML::Node & node) const
{
  bool writeMulti = blocks::WriteStructure::WRITE_MULTI_DEFAULT;
  if(node[kw::BLOCKS__WRITE_STRUCTURES__MULTI])
  {
    try
    {
      writeMulti = node[kw::BLOCKS__WRITE_STRUCTURES__MULTI].as<bool>();
    }
    catch(const YAML::Exception & /*e*/) {}
  }

  blockOut.reset(
    ManagedBlockType::create(sstbx::makeUniquePtr(new blocks::WriteStructure(writeMulti)))
  );
  return true;
}

} // namespace common
} // namespace factory


#endif // SP_USE_YAML
