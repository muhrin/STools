/*
 * BlockFactory.h
 *
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef BLOCK_FACTORY_H
#define BLOCK_FACTORY_H

// INCLUDES /////////////////////////////////////////////
#include "StructurePipe.h"

#include <string>

#include <spl/SSLib.h>
#include <spl/factory/SsLibFactoryYaml.h>
#include <spl/potential/OptimisationSettings.h>

// Local includes
#include "StructurePipe.h"
#include "SpTypes.h"
#include "factory/YamlSchema.h"

// FORWARD DECLARATIONS ////////////////////////////////////

namespace spipe {
namespace factory {

class BlockFactory
{
public:
  bool
  createBlock(BlockHandle * const blockOut,
      const blocks::BuildStructures & options) const;
  bool
  createBlock(BlockHandle * const blockOut,
      const blocks::Clone & options) const;
  bool
  createBlock(BlockHandle * const blockOut,
      const blocks::CutAndPaste & options) const;
  bool
  createBlock(BlockHandle * const blockOut,
      const blocks::FindSymmetryGroup & options) const;
  bool
  createBlock(BlockHandle * const blockOut,
      const blocks::GeomOptimise & options) const;
#ifdef SPL_WITH_CGAL
  bool
  createBlock(BlockHandle * const blockOut,
      const blocks::KeepStableCompositions & options) const;
#endif
  bool
  createBlock(BlockHandle * const blockOut,
      const blocks::KeepTopN & options) const;
  bool
  createBlock(BlockHandle * const blockOut,
      const blocks::KeepWithinXPercent & options) const;
  bool
  createBlock(BlockHandle * const blockOut, const std::string & toLoad) const;
  bool
  createBlock(BlockHandle * const blockOut,
      const blocks::NiggliReduce & options) const;
  bool
  createBlock(BlockHandle * const blockOut,
      const blocks::PasteFragment & options) const;
  bool
  createBlock(BlockHandle * const blockOut,
      const blocks::RemoveDuplicates & options) const;
  bool
  createBlock(BlockHandle * const blockOut,
      blocks::RunPotentialParamsQueue & options) const;
  bool
  createBlock(BlockHandle * const blockOut,
      blocks::SearchStoichiometries & options) const;
  bool
  createBlock(BlockHandle * const blockOut,
      blocks::SeparateAtoms & options) const;
  bool
  createBlock(BlockHandle * const blockOut,
      blocks::SweepPotentialParams & options) const;
  bool
  createBlock(BlockHandle * const blockOut,
      const blocks::WriteStructures & options) const;

private:
  spl::factory::Factory mySplFactory;
};

}
}

#endif /* BLOCK_FACTORY_H */

