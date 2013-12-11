/*
 * StructureData.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "common/StructureData.h"

#include <pipelib/pipelib.h>

// Local includes
#include "common/SharedData.h"
#include "spl/common/Structure.h"
#include "spl/io/ResourceLocator.h"

// NAMESPACES ////////////////////////////////

namespace spipe {
namespace common {
namespace fs = ::boost::filesystem;
namespace ssc = ::spl::common;
namespace ssio = ::spl::io;
namespace ssu = ::spl::utility;
namespace structure_properties = ssc::structure_properties;

StructureData &
StructureData::operator =(const StructureData & rhs)
{
  objectsStore = rhs.objectsStore;
  if(rhs.myStructure.get())
    myStructure.reset(new ssc::Structure(*rhs.myStructure));
  return *this;
}

ssc::Structure *
StructureData::getStructure() const
{
  return myStructure.get();
}

ssc::Structure &
StructureData::setStructure(ssc::types::StructurePtr structure)
{
#ifdef SSLIB_USE_CPP11
  myStructure = ::std::move(structure);
#else
  myStructure = structure;
#endif
  return *myStructure.get();
}

ssc::Structure &
StructureData::setStructure(ssio::StructuresContainer::auto_type structure)
{
  myStructure.reset(structure.release());
  return *myStructure.get();
}

ssio::ResourceLocator
StructureData::getRelativeSavePath(const ::boost::filesystem::path & relativeTo) const
{
  ssio::ResourceLocator relativeLocator;

  const ssc::Structure * const structure = getStructure();

  // If no structure, return empty locator
  if(!structure)
    return relativeLocator;

  const ssio::ResourceLocator * lastSaved = structure->getProperty(
      structure_properties::io::LAST_ABS_FILE_PATH);

  if(lastSaved)
  {
    relativeLocator = *lastSaved;
    fs::path relativePath = lastSaved->path();
    // Make the path relative if necessary
    relativePath = ssio::make_relative(relativeTo, relativePath);
    relativeLocator.setPath(relativePath);
  }

  return relativeLocator;
}

}
}
