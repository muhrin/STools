/*
 * UniqueStructureSet.h
 *
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef UNIQUE_STRUCTURE_SET_H
#define UNIQUE_STRUCTURE_SET_H

// INCLUDES /////////////////////////////////////////////
#include "IStructureSet.h"

#include <map>

#include <boost/shared_ptr.hpp>

#include "common/Structure.h"
#include "utility/IBufferedComparator.h"



// FORWARD DECLARATIONS ////////////////////////////////////
namespace sstbx
{
namespace utility
{
class IStructureComparator;
}
}


namespace sstbx { namespace utility {

class UniqueStructureSet : public IStructureSet
{
private:

	struct StructureMetadata
	{
    StructureMetadata(const IBufferedComparator::DataHandle & compHandle);

		/** The number of times this structure has been found */
		size_t				                  timesFound;
		IBufferedComparator::DataHandle compHandle;
	};

public:

	typedef std::pair<sstbx::common::Structure *, bool> ReturnPair;

	UniqueStructureSet(const IStructureComparator & comparator);
	virtual ~UniqueStructureSet();

	// From IStructureSet //

	virtual const std::pair<sstbx::common::Structure *, bool>
		insert(sstbx::common::Structure * const str);

	virtual void clear();

	// End from IStructureSet //

private:

	typedef std::map<sstbx::common::Structure *, StructureMetadata> StructureSet;

  const ::boost::shared_ptr<IBufferedComparator> myComparator;

	StructureSet myStructures;
};


}
}


#endif /* UNIQUE_STRUCTURE_SET_H */
