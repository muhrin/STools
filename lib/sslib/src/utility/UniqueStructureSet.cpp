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


namespace sstbx
{
namespace utility
{


// StructureMetadata implementation ///////

UniqueStructureSet::StructureMetadata::StructureMetadata(
	const IBufferedComparator::DataHandle & _compHandle):
compHandle(_compHandle),
timesFound(0)
{}

// UniqueStructureSet implementation

UniqueStructureSet::UniqueStructureSet(const IStructureComparator & comparator):
myComparator(comparator.generateBuffered())
{}

UniqueStructureSet::~UniqueStructureSet()
{
  BOOST_FOREACH(StructureSet::value_type & strDat, myStructures)
	{
		myComparator->releaseComparisonData(strDat.second.compHandle);
	}
}

const std::pair<sstbx::common::Structure *, bool>
UniqueStructureSet::insert(sstbx::common::Structure * const str)
{
	// First creat the comparison data instance for this structure
	IBufferedComparator::DataHandle compHandle = myComparator->generateComparisonData(*str);

	sstbx::common::Structure * similarTo = NULL;
	bool unique = true;
  BOOST_FOREACH(StructureSet::value_type strDat, myStructures)
	{
		if(myComparator->areSimilar(compHandle, strDat.second.compHandle))
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
      StructureSet::value_type(
			str,
			StructureMetadata(compHandle)
			)
		);
	}
  else
  {
    // Tell the comparator the remove the comparison data
    myComparator->releaseComparisonData(compHandle);
  }

	return ReturnPair(unique ? str : similarTo, unique);
}

void UniqueStructureSet::clear()
{
  BOOST_FOREACH(StructureSet::value_type & strDat, myStructures)
	{
		myComparator->releaseComparisonData(strDat.second.compHandle);
	}
	myStructures.clear();
}

}
}
