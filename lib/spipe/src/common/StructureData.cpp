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
namespace ssu = ::sstbx::utility;


::sstbx::utility::Key< ::boost::filesystem::path> StructureObjectKeys::LAST_ABS_SAVE_PATH;
::sstbx::utility::Key<unsigned int> StructureObjectKeys::SPACEGROUP_NUMBER;
::sstbx::utility::Key< ::std::string> StructureObjectKeys::SPACEGROUP_SYMBOL;
::sstbx::utility::Key< double > StructureObjectKeys::PRESSURE_INTERNAL;

ssc::Structure * StructureData::getStructure() const
{
  return myStructure.get();
}

ssc::Structure & StructureData::setStructure(sstbx::UniquePtr<ssc::Structure>::Type structure)
{
#ifdef SSLIB_USE_CPP11
  myStructure = ::std::move(structure);
#else
	myStructure = structure;
#endif
  return *myStructure.get();
}

::boost::filesystem::path
StructureData::getRelativeSavePath(const ::spipe::SpPipelineTyp & pipeline) const
{
  fs::path relativePath;

  const fs::path * lastSaved = objectsStore.find(common::StructureObjectKeys::LAST_ABS_SAVE_PATH);

  if(lastSaved)
  {
    relativePath = *lastSaved;
    if(ssu::fs::isAbsolute(relativePath))
    {
      relativePath = ssu::fs::make_relative(pipeline.getSharedData().getOutputPath(), relativePath);
    }
  }

  return relativePath;
}

}}
