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
  StructureDescriptionPtr structureDescription
):
SpBlock("Generate Random structures"),
myNumToGenerate(numToGenerate),
myAtomsMultiplierGenerate(0.0),
myFixedNumGenerate(true),
myStructureGenerator(new ssbc::DefaultCrystalGenerator(true /*use extrusion method*/)),
myStructureDescription(structureDescription)
{
}

RandomStructure::RandomStructure(
  const float atomsMultiplierGenerate,
  StructureDescriptionPtr structureDescription
):
SpBlock("Generate Random structures"),
myNumToGenerate(0),
myAtomsMultiplierGenerate(atomsMultiplierGenerate),
myFixedNumGenerate(false),
myStructureGenerator(new ssbc::DefaultCrystalGenerator(true /*use extrusion method*/)),
myStructureDescription(structureDescription)
{
}

void RandomStructure::start()
{
	using ::spipe::common::StructureData;

  const ssbc::StructureDescription * const strDesc = getStructureDescription();

  if(strDesc)
  {
    unsigned int numToGenerate = myFixedNumGenerate ? myNumToGenerate : 100;
  	
    float totalAtomsGenerated = 0.0;

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
}


void RandomStructure::in(::spipe::common::StructureData & data)
{
  const ssbc::StructureDescription * const strDesc = getStructureDescription();

  if(!strDesc)
  {
    out(data);
    return;
  }

	// Create the random structure
  ssc::StructurePtr str = myStructureGenerator->generateStructure(*strDesc, getRunner()->memory().global().getSpeciesDatabase());

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

const ::sstbx::build_cell::StructureDescription *
RandomStructure::getStructureDescription() const
{
  const ssbc::StructureDescription * strDesc = NULL;
  if(myStructureDescription.get())
    strDesc = myStructureDescription.get();
  else
  {
    // Try finding one in shared memory
    if(getRunner())
      strDesc = getRunner()->memory().shared().getStructureDescription();
  }

  return strDesc;
}

}
}
