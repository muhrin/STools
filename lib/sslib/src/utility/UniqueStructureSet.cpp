/*
 * UniqueStructureSet.cpp
 *
 *  Created on: Aug 17, 2011
 *      Author: muhrin
 */

// INCLUDES /////////////////////////////////////
#include "utility/UniqueStructureSet.h"

#include <boost/foreach.hpp>

#include "utility/IStructureComparator.h"


namespace sstbx {
namespace utility {


// StructureMetadata implementation ///////

UniqueStructureSet::StructureMetadata::StructureMetadata():
timesFound(1)
{}

// UniqueStructureSet implementation

UniqueStructureSet::UniqueStructureSet(const IStructureComparator & comparator):
myComparator(comparator.generateBuffered())
{}

const std::pair<sstbx::common::Structure *, bool>
UniqueStructureSet::insert(sstbx::common::Structure * const str)
{
	sstbx::common::Structure * similarTo = NULL;
	bool unique = true;
  BOOST_FOREACH(StructureSet::value_type strDat, myStructures)
	{
		if(myComparator->areSimilar(*str, *strDat.first))
		{
      ++strDat.second.timesFound;
			similarTo = strDat.first;
			unique = false;
			break;
		}
	}

	// If it is not like any of those we have already then insert it
	if(unique)
	{
		myStructures.insert(
      StructureSet::value_type(str, StructureMetadata())
		);
	}
  else
  {
    // Tell the comparator the remove the comparison data for this structure as we
    // are not storing it.
    myComparator->releaseComparisonDataFor(*str);
  }

	return ReturnPair(unique ? str : similarTo, unique);
}

void UniqueStructureSet::clear()
{
  BOOST_FOREACH(StructureSet::value_type & strDat, myStructures)
	{
		myComparator->releaseComparisonDataFor(*strDat.first);
	}
	myStructures.clear();
}

}
}
