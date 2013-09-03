/*
 * RemoveDuplicates.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "blocks/RemoveDuplicates.h"
#include "StructurePipe.h"

#include <map>

#include <boost/foreach.hpp>

#include <spl/common/Structure.h>

#include "common/StructureData.h"

// NAMESPACES ////////////////////////////////

namespace spipe {
namespace blocks {

namespace ssc = ::spl::common;
namespace ssu = ::spl::utility;
namespace structure_properties = ssc::structure_properties;

RemoveDuplicates::RemoveDuplicates(ssu::IStructureComparatorPtr comparator) :
    Block("Remove duplicates"), myStructureSet(comparator)
{
}

RemoveDuplicates::RemoveDuplicates(
    const spl::utility::IStructureComparator & comparator) :
    Block("Remove duplicates"), myStructureSet(comparator)
{
}

void
RemoveDuplicates::in(common::StructureData * const data)
{
  if(!data->getStructure())
  {
    out(data);
    return;
  }

  const StructureSet::insert_return_type result = myStructureSet.insert(data,
      *data->getStructure());

  if(result.second)
  {
    // Inserted
    data->getStructure()->setProperty(
        structure_properties::searching::TIMES_FOUND,
        static_cast<unsigned int>(1));
  }
  else
  {
    // The structure is not unique so discard it
    drop(data);

    // Up the 'times found' counter on the original structure
    ::spipe::common::StructureData * const origStrData = *result.first;
    unsigned int * const timesFound = origStrData->getStructure()->getProperty(
        structure_properties::searching::TIMES_FOUND);
    if(timesFound)
      *timesFound += 1;
    else
      origStrData->getStructure()->setProperty(
          structure_properties::searching::TIMES_FOUND,
          static_cast< unsigned int>(1));
  }
}

void
RemoveDuplicates::pipelineFinishing()
{
  myStructureSet.clear();
}

size_t
RemoveDuplicates::release()
{
  const size_t numUnique = myStructureSet.size();

  BOOST_FOREACH(common::StructureData * const data, myStructureSet)
  {
    out(data);
  }
  myStructureSet.clear();

  return numUnique;
}

bool
RemoveDuplicates::hasData() const
{
  return !myStructureSet.empty();
}

}
}
