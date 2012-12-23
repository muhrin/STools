/*
 * WriteStructure.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "blocks/WriteStructure.h"

#include <pipelib/IPipeline.h>

// From SSTbx
#include <common/Structure.h>
#include <io/StructureReadWriteManager.h>
#include <io/BoostFilesystem.h>

// From local
#include "common/SharedData.h"
#include "common/StructureData.h"
#include "common/UtilityFunctions.h"


// NAMESPACES ////////////////////////////////


namespace spipe {
namespace blocks {

namespace ssc = ::sstbx::common;
namespace fs = ::boost::filesystem;
namespace ssu = ::sstbx::utility;

WriteStructure::WriteStructure(const ::sstbx::io::StructureReadWriteManager & writerManager):
pipelib::Block<StructureDataTyp, SharedDataTyp>("Write structures"),
myWriterManager(writerManager)
{}

void WriteStructure::in(::spipe::common::StructureData & data)
{
  ssc::Structure * const structure = data.getStructure();

	// Check if the structure has a name already, otherwise give it one
	if(structure->getName().empty())
	{
		structure->setName(::spipe::common::generateUniqueName());
	}

	// Create the path to store the structure
	fs::path p(structure->getName() + ".res");

  // Prepend the pipe output path
  p = myPipeline->getSharedData().getOutputPath() / p;
	
  if(!myWriterManager.writeStructure(*data.getStructure(), p, myPipeline->getGlobalData().getSpeciesDatabase()))
  {
    // TODO: Produce error
  }

	myOutput->in(data);
}

}
}

