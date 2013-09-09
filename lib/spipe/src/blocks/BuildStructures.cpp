/*
 * BuildStructures.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "blocks/BuildStructures.h"

#include <cmath>

#include <boost/optional.hpp>

#include <spl/SSLib.h>
#include <spl/build_cell/AtomsDescription.h>
#include <spl/build_cell/GenerationOutcome.h>
#include <spl/build_cell/IStructureGenerator.h>
#include <spl/common/Constants.h>
#include <spl/common/Structure.h>
#include <spl/common/Types.h>
#include <spl/utility/UtilFunctions.h>

// Local includes
#include "common/PipeFunctions.h"
#include "common/UtilityFunctions.h"

// NAMESPACES ////////////////////////////////

namespace spipe {
namespace blocks {

namespace ssbc = ::spl::build_cell;
namespace ssc = ::spl::common;
namespace ssu = ::spl::utility;

BuildStructures::BuildStructures(const int numToGenerate,
    IStructureGeneratorPtr structureGenerator) :
    Block("Generate Random structures"), myNumToGenerate(numToGenerate), myAtomsMultiplierGenerate(
        0.0), myFixedNumGenerate(true), myStructureGenerator(structureGenerator)
{
}

BuildStructures::BuildStructures(const float atomsMultiplierGenerate,
    IStructureGeneratorPtr structureGenerator) :
    Block("Generate Random structures"), myNumToGenerate(0), myAtomsMultiplierGenerate(
        atomsMultiplierGenerate), myFixedNumGenerate(false), myStructureGenerator(
        structureGenerator)
{
}

void
BuildStructures::start()
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
      outcome = generator->generateStructure(str,
          getEngine()->globalData().getSpeciesDatabase());

      if(outcome.success() && str.get() && str->getNumAtoms() != 0)
      {
        StructureData * const data = getEngine()->createData();
        data->setStructure(str);

        data->getStructure()->setName(generateStructureName(i));

        if(!myFixedNumGenerate)
        {
          totalAtomsGenerated += static_cast< float>(str->getNumAtoms());
          numToGenerate = static_cast< int>(std::ceil(
              myAtomsMultiplierGenerate * totalAtomsGenerated
                  / static_cast< float>(i)));
        }

        // Send it down the pipe
        out(data);
      }
    }
  }
}

void
BuildStructures::in(::spipe::common::StructureData * const data)
{
  ssbc::IStructureGenerator * const generator = getStructureGenerator();
  if(!generator)
  {
    out(data);
    return;
  }

#ifdef SP_ENABLE_THREAD_AWARE
  myBuildStructuresMutex.lock();
#endif

  // Create the random structure
  ssc::StructurePtr str;
  const ssbc::GenerationOutcome outcome = generator->generateStructure(str,
      getEngine()->globalData().getSpeciesDatabase());

#ifdef SP_ENABLE_THREAD_AWARE
  myBuildStructuresMutex.unlock();
#endif

  if(outcome.success() && str.get())
  {
    data->setStructure(str);

    // Build up the name
    if(data->getStructure()->getName().empty())
      data->getStructure()->setName(
          common::generateStructureName(getEngine()->globalData()));

    // Send it down the pipe
    out(data);
  }
  else
    drop(data);
}

::spl::build_cell::IStructureGenerator *
BuildStructures::getStructureGenerator()
{
  ssbc::IStructureGenerator * generator = NULL;
  if(myStructureGenerator.get())
    generator = myStructureGenerator.get();
  else
  {
    // Try finding one in shared memory
    if(getEngine())
      generator = getEngine()->sharedData().getStructureGenerator();
  }

  return generator;
}

::std::string
BuildStructures::generateStructureName(const size_t structureNum) const
{
  // Build up the name
  ::std::stringstream ss;
  ss << common::generateStructureName(getEngine()->globalData()) << "-"
      << structureNum;
  return ss.str();
}

}
}
