/*
 * IBufferedComparator.h
 *
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef I_BUFFERED_COMPARATOR_H
#define I_BUFFERED_COMPARATOR_H

// INCLUDES /////////////////////////////////////////////

#include <limits>

// FORWARD DECLARATIONS ////////////////////////////////////
namespace sstbx
{
namespace common
{
class Structure;
}
namespace utility
{
class IStructureComparator;
}
}

namespace sstbx
{
namespace utility
{

class IBufferedComparator
{
public:
  typedef unsigned int DataHandle;
  static const DataHandle UNINITIALISED;

  virtual ~IBufferedComparator() {}

	virtual double compareStructures(
		const DataHandle & str1Data,
		const DataHandle & str2Data) const = 0;

	virtual bool areSimilar(
		const DataHandle & str1Data,
		const DataHandle & str2Data) const = 0;

  virtual const DataHandle generateComparisonData(const ::sstbx::common::Structure & str) = 0;
  virtual void releaseComparisonData(DataHandle & handle) = 0;

  virtual const IStructureComparator & getComparator() const = 0;
};

}
}

#endif /* I_BUFFERED_COMPARATOR_H */
