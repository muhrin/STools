/*
 * StructureData.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "common/StructureData.h"

#include <pipelib/IPipeline.h>

// Local includes
#include "common/SharedData.h"
#include "common/Structure.h"


// NAMESPACES ////////////////////////////////

namespace spipe {
namespace common {
namespace fs = ::boost::filesystem;
namespace ssc = ::sstbx::common;
namespace ssio = ::sstbx::io;
namespace ssu = ::sstbx::utility;
namespace structure_properties = ssc::structure_properties;


ssc::Structure * StructureData::getStructure() const
{
  return myStructure.get();
}

ssc::Structure & StructureData::setStructure(ssc::types::StructurePtr structure)
{
#ifdef SSLIB_USE_CPP11
  myStructure = ::std::move(structure);
#else
	myStructure = structure;
#endif
  return *myStructure.get();
}

ssc::Structure & StructureData::setStructure(ssio::StructuresContainer::auto_type structure)
{
  myStructure.reset(structure.release());
  return *myStructure.get();
}

::boost::filesystem::path
StructureData::getRelativeSavePath(const ::spipe::SpPipelineTyp & pipeline) const
{
  fs::path relativePath;

  const ssc::Structure * const structure = getStructure();

  // If no structure, return empty path
  if(!structure)
    return relativePath;

  const fs::path * lastSaved = structure->getProperty(structure_properties::io::LAST_ABS_FILE_PATH);

  if(lastSaved)
  {
    relativePath = *lastSaved;
    if(ssio::isAbsolute(relativePath))
    {
      relativePath = ssio::make_relative(pipeline.getSharedData().getOutputPath(), relativePath);
    }
  }

  return relativePath;
}

}}
