/*
 * StructureData.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "common/StructureData.h"

#include "common/Structure.h"


// NAMESPACES ////////////////////////////////

namespace spipe
{
namespace common
{

const ::sstbx::utility::Key< ::boost::filesystem::path> StructureObjectKeys::LAST_ABS_SAVE_PATH;

StructureData::StructureData():
myStructure(NULL)
{}

StructureData::~StructureData()
{
	if(myStructure)
		delete myStructure;
}

sstbx::common::Structure * StructureData::getStructure() const
{
	return myStructure;
}

void StructureData::setStructure(sstbx::common::Structure * const structure)
{
	myStructure = structure;
}

}}
