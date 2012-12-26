/*
 * WriteStructure.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "blocks/WriteStructure.h"

// From SSTbx
#include <common/Structure.h>
#include <io/StructureReadWriteManager.h>
#include <io/BoostFilesystem.h>
#include <io/ResourceLocator.h>

// From local
#include "common/SharedData.h"
#include "common/StructureData.h"
#include "common/UtilityFunctions.h"


// NAMESPACES ////////////////////////////////


namespace spipe {
namespace blocks {

namespace fs = ::boost::filesystem;
namespace ssc = ::sstbx::common;
namespace ssio = ::sstbx::io;
namespace ssu = ::sstbx::utility;

WriteStructure::WriteStructure(const bool writeMultiStructure):
SpBlock("Write structures"),
myWriteMultiStructure(writeMultiStructure)
{}

void WriteStructure::in(::spipe::common::StructureData & data)
{
  const ssio::StructureReadWriteManager & rwMan = getRunner()->memory().global().getStructureIo();
  if(rwMan.getDefaultWriter())
  {
    common::SharedData & shared = getRunner()->memory().shared();
    ssc::Structure * const structure = data.getStructure();

	  // Check if the structure has a name already, otherwise give it one
	  if(structure->getName().empty())
	  {
		  structure->setName(::spipe::common::generateUniqueName());
	  }

	  // Create the path to store the structure
    fs::path p(shared.getOutputPath(*getRunner()));
    if(!myWriteMultiStructure || !rwMan.getDefaultWriter()->multiStructureSupport())
    {
      p /= fs::path(structure->getName());
    }
    ssio::ResourceLocator saveLocation(p, structure->getName());
  	
    if(!rwMan.writeStructure(*data.getStructure(), saveLocation, getRunner()->memory().global().getSpeciesDatabase()))
    {
      // TODO: Produce error
    }
  }
	out(data);
}

}
}

