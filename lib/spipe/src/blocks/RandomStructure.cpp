/*
 * RandomStructure.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "blocks/RandomStructure.h"

#include <cmath>

#include <boost/optional.hpp>

// From SSTbx
#include <SSLib.h>
#include <build_cell/AtomsDescription.h>
#include <build_cell/ConstStructureDescriptionVisitor.h>
#include <build_cell/DefaultCrystalGenerator.h>
#include <build_cell/IStructureGenerator.h>
#include <common/Constants.h>
#include <common/Structure.h>
#include <common/Types.h>
#include <utility/UtilFunctions.h>

// Local includes
#include "common/UtilityFunctions.h"

// NAMESPACES ////////////////////////////////


namespace spipe {
namespace blocks {

namespace ssbc = ::sstbx::build_cell;
namespace ssc = ::sstbx::common;
namespace ssu = ::sstbx::utility;

RandomStructure::RandomStructure(
  const unsigned int numToGenerate,
  const ::boost::shared_ptr<const ::sstbx::build_cell::StructureDescription > & structureDescription):
SpBlock("Generate Random structures"),
myNumToGenerate(numToGenerate),
myAtomsMultiplierGenerate(0.0),
myFixedNumGenerate(true),
myStructureGenerator(new ssbc::DefaultCrystalGenerator(true /*use extrusion method*/)),
myStructureDescription(structureDescription),
myUseSharedDataStructureDesc(!structureDescription.get())
{
}

RandomStructure::RandomStructure(
  const float atomsMultiplierGenerate,
  const ::boost::shared_ptr<const ::sstbx::build_cell::StructureDescription > & structureDescription):
SpBlock("Generate Random structures"),
myNumToGenerate(0),
myAtomsMultiplierGenerate(atomsMultiplierGenerate),
myFixedNumGenerate(false),
myStructureGenerator(new ssbc::DefaultCrystalGenerator(true /*use extrusion method*/)),
myStructureDescription(structureDescription),
myUseSharedDataStructureDesc(!structureDescription.get())
{
}

void RandomStructure::pipelineStarting()
{
  // TODO: Put structure description initialisation stuff here
}

void RandomStructure::start()
{
	using ::spipe::common::StructureData;
  unsigned int numToGenerate = myFixedNumGenerate ? myNumToGenerate : 100;
	
  float totalAtomsGenerated = 0.0;
  initDescriptions();
  for(size_t i = 0; i < numToGenerate; ++i)
  {
	  // Create the random structure
    ssc::StructurePtr str = myStructureGenerator->generateStructure(*myStructureDescription, getRunner()->memory().global().getSpeciesDatabase());

	  if(str.get())
	  {
      StructureData & data = getRunner()->createData();
		  data.setStructure(str);

		  // Build up the name
			std::stringstream ss;
			ss << ssu::generateUniqueName() << "-" << i;
			data.getStructure()->setName(ss.str());

      if(!myFixedNumGenerate)
      {
        totalAtomsGenerated += static_cast<float>(str->getNumAtoms());
        numToGenerate = static_cast<unsigned int>(std::ceil(
          myAtomsMultiplierGenerate * totalAtomsGenerated /
          static_cast<float>(i))
          );
      }

		  // Send it down the pipe
		  out(data);
	  }
  }	
}


void RandomStructure::in(::spipe::common::StructureData & data)
{
  initDescriptions();

	// Create the random structure
  ssc::StructurePtr str = myStructureGenerator->generateStructure(*myStructureDescription, getRunner()->memory().global().getSpeciesDatabase());

	if(str.get())
	{
		data.setStructure(str);

		// Build up the name
		if(!data.getStructure()->getName().empty())
		{
			std::stringstream ss;
			ss << ssu::generateUniqueName();
			data.getStructure()->setName(ss.str());
		}

		// Send it down the pipe
		out(data);
	}
	else
		getRunner()->dropData(data);
}

void RandomStructure::initDescriptions()
{
  const common::SharedData & sharedDat = getRunner()->memory().shared();
  if(myUseSharedDataStructureDesc)
  {
    if(sharedDat.getStructureDescription())
    {
      myStructureDescription = sharedDat.getStructureDescription();
    }
    else
    {
      // TODO: Throw some kind of exception, or emit error
    }
  }
}

}
}
