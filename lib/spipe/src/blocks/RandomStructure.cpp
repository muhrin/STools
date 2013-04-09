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
#include <build_cell/GenerationOutcome.h>
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
  const int numToGenerate,
  IStructureGeneratorPtr structureGenerator
):
SpBlock("Generate Random structures"),
myNumToGenerate(numToGenerate),
myAtomsMultiplierGenerate(0.0),
myFixedNumGenerate(true),
myStructureGenerator(structureGenerator)
{
}

RandomStructure::RandomStructure(
  const float atomsMultiplierGenerate,
  IStructureGeneratorPtr structureGenerator
):
SpBlock("Generate Random structures"),
myNumToGenerate(0),
myAtomsMultiplierGenerate(atomsMultiplierGenerate),
myFixedNumGenerate(false),
myStructureGenerator(structureGenerator)
{
}

void RandomStructure::start()
{
	using ::spipe::common::StructureData;

  ssbc::IStructureGenerator * const generator = getStructureGenerator();

  if(generator)
  {
    int numToGenerate = myFixedNumGenerate ? myNumToGenerate : 100;
  	
    float totalAtomsGenerated = 0.0;
    ssbc::GenerationOutcome outcome;
    for(int i = 0; i < numToGenerate; ++i)
    {
	    // Create the random structure
      ssc::StructurePtr str;
      outcome = generator->generateStructure(str, getRunner()->memory().global().getSpeciesDatabase());

	    if(outcome.success() && str.get())
	    {
        StructureData & data = getRunner()->createData();
		    data.setStructure(str);

			  data.getStructure()->setName(generateStructureName(*getRunner(), i));

        if(!myFixedNumGenerate)
        {
          totalAtomsGenerated += static_cast<float>(str->getNumAtoms());
          numToGenerate = static_cast<int>(std::ceil(
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
  ssbc::IStructureGenerator * const generator = getStructureGenerator();
  if(!generator)
  {
    out(data);
    return;
  }

	// Create the random structure
  ssc::StructurePtr str;
  const ssbc::GenerationOutcome outcome = 
    generator->generateStructure(str, getRunner()->memory().global().getSpeciesDatabase());

	if(outcome.success() && str.get())
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

::sstbx::build_cell::IStructureGenerator *
RandomStructure::getStructureGenerator()
{
  ssbc::IStructureGenerator * generator = NULL;
  if(myStructureGenerator.get())
    generator = myStructureGenerator.get();
  else
  {
    // Try finding one in shared memory
    if(getRunner())
      generator = getRunner()->memory().shared().getStructureGenerator();
  }

  return generator;
}

::std::string RandomStructure::generateStructureName(const SpRunnerAccess & runner, const size_t structureNum) const
{
  // Build up the name
  ::std::stringstream ss;
  ss << ssu::generateUniqueName() << "-" << structureNum;
  return ss.str();
}

}
}
