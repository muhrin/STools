/*
 * Factory.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "factory/Factory.h"

// SSLib includes
#include <potential/Types.h>
#include <utility/UtilityFwd.h>
#include <factory/SsLibElements.h>

// Local includes
#include "blocks/DetermineSpaceGroup.h"
#include "blocks/LowestFreeEnergy.h"
#include "blocks/NiggliReduction.h"
#include "blocks/ParamPotentialGo.h"
#include "blocks/PotentialParamSweep.h"
#include "blocks/RandomStructure.h"
#include "blocks/RemoveDuplicates.h"
#include "blocks/WriteStructure.h"
#include "common/CommonData.h"
#include "common/StructureData.h"
#include "common/SharedData.h"
#include "factory/MapEntries.h"


// NAMESPACES ////////////////////////////////

namespace spipe {
namespace factory {

// Alias for accessing keywords namespace
namespace ssf     = ::sstbx::factory;
namespace spb     = ::spipe::blocks;
namespace ssbc    = ::sstbx::build_cell;
namespace ssio    = ::sstbx::io;
namespace ssp     = ::sstbx::potential;
namespace ssu     = ::sstbx::utility;

bool Factory::createDetermineSpaceGroupBlock(BlockPtr & blockOut) const
{
  blockOut.reset(new blocks::DetermineSpaceGroup());
  return true;
}

bool Factory::createLowestEnergyBlock(BlockPtr & blockOut, const OptionsMap & options) const
{
  const double * const keepWithin = options.find(KEEP_WITHIN);
  if(keepWithin)
  {
    blockOut.reset(new blocks::LowestFreeEnergy(*keepWithin));
    return true;
  }

  const size_t * const keepTop = options.find(KEEP_TOP);
  if(keepTop)
  {
    blockOut.reset(new blocks::LowestFreeEnergy(*keepTop));
    return true;
  }
  
  return false;
}

bool Factory::createNiggliReduceBlock(BlockPtr & blockOut) const
{
  blockOut.reset(new blocks::NiggliReduction());
  return true;
}

bool Factory::createParamSweepBlock(
  BlockPtr & blockOut,
  const OptionsMap & options,
  PipePtr subPipe
) const
{
  const ::std::vector< ::std::string> * const paramStrings = options.find(PARAM_RANGE);
  if(!paramStrings)
    return false;
  common::ParamRange paramRange;
  paramRange.fromStrings(*paramStrings);

  blockOut.reset(new blocks::PotentialParamSweep(paramRange, subPipe));
  return true;
}

bool Factory::createPotentialGeomOptimiseBlock(
  BlockPtr & blockOut,
  const OptionsMap & optimiserOptions,
  const OptionsMap * const potentialOptions,
  const ssp::OptimisationSettings * optimisationSettings
) const
{
  ssp::IGeomOptimiserPtr optimiser = mySsLibFactory.createGeometryOptimiser(optimiserOptions, potentialOptions);

  bool potentialIsParameterisable =
    optimiser->getPotential() && optimiser->getPotential()->getParameterisable();

  if(!optimiser.get())
    return false;

  if(optimisationSettings)
  {
    if(potentialIsParameterisable)
      blockOut.reset(new blocks::ParamPotentialGo(optimiser, *optimisationSettings));
    else
      blockOut.reset(new blocks::PotentialGo(optimiser, *optimisationSettings));
  }
  else
  {
    if(potentialIsParameterisable)
      blockOut.reset(new blocks::ParamPotentialGo(optimiser));
    else
      blockOut.reset(new blocks::PotentialGo(optimiser));
  }

  return true;
}

bool Factory::createRandomStructureBlock(
  BlockPtr & blockOut,
  const OptionsMap & options
) const
{
  // Try to construct a structure generator
  ssbc::IStructureGeneratorPtr generator(mySsLibFactory.createStructureBuilder(options));
  if(!generator.get())
    return false;

  const int * const numToGenerate = options.find(NUM);
  if(!numToGenerate)
    return false;

  blockOut.reset(new blocks::RandomStructure(*numToGenerate, generator));

  return true;
}

bool Factory::createRemoveDuplicatesBlock(
  BlockPtr & blockOut,
  const OptionsMap & options
) const
{
  // Check for setting relating to the comparator
  const OptionsMap * const comparatorOptions = options.find(ssf::COMPARATOR);
  if(!comparatorOptions)
    return false;

  ssu::IStructureComparatorPtr comparator(
    mySsLibFactory.createStructureComparator(*comparatorOptions)
  );
  
  if(!comparator.get())
    return false;

  blockOut.reset(new blocks::RemoveDuplicates(comparator));

  return true;
}

bool
Factory::createWriteStructuresBlock(
  BlockPtr & blockOut,
  const OptionsMap & options
) const
{
  ::sstbx::UniquePtr<blocks::WriteStructure>::Type
    writeStructure(new blocks::WriteStructure());

  const bool * const multiWrite = options.find(MULTI_WRITE);
  if(multiWrite)
    writeStructure->setWriteMulti(*multiWrite);

  const ::std::string * const fileType = options.find(FILE_TYPE);
  if(fileType)
    writeStructure->setFileType(*fileType);

  // Transfer ownership
  blockOut = writeStructure;
  return true;
}

} // namespace common
} // namespace factory
