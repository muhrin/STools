/*
 * GenericBufferedComparator.h
 *
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */


#ifndef GENERIC_BUFFERED_COMPARATOR_H
#define GENERIC_BUFFERED_COMPARATOR_H

/* TODO **********************************************
- Add method for comparing handle to all 
- Maybe make the comparison data structure a growing array where
  removed elements just get forgotten about or the handles are a
  class that is updated by subtracting the appropriate amount of
  elements
******************************************************/

// INCLUDES /////////////////////////////////////////////

#include <map>
#include <memory>

//#include <boost/ptr_container/ptr_map.hpp>
#include <boost/shared_ptr.hpp>

#include "utility/IBufferedComparator.h"

// FORWARD DECLARATIONS ////////////////////////////////////
namespace sstbx
{
namespace common
{
	class Structure;
}
}

namespace sstbx
{
namespace utility
{

template <class ComparatorTyp, class DataTyp>
class GenericBufferedComparator : public IBufferedComparator
{
public:

  GenericBufferedComparator(const ComparatorTyp & comparator);
  virtual ~GenericBufferedComparator() {}

	virtual double compareStructures(
    const IBufferedComparator::DataHandle & str1Data,
		const IBufferedComparator::DataHandle & str2Data) const;

	virtual bool areSimilar(
		const IBufferedComparator::DataHandle & str1Data,
		const IBufferedComparator::DataHandle & str2Data) const;

  virtual const DataHandle generateComparisonData(const ::sstbx::common::Structure & str);
  virtual void releaseComparisonData(IBufferedComparator::DataHandle & handle);

  virtual const ComparatorTyp & getComparator() const;

private:

  // TODO: Change this from shared_ptr to ptr_multimap
  //typedef typename ::boost::ptr_multimap<DataHandle, const DataTyp> DataMap;
  typedef typename ::std::map<DataHandle, ::boost::shared_ptr<const DataTyp> > DataMap;

  const ComparatorTyp &   myComparator;
  DataMap                 myDataMap;
  size_t                  myTotalData;
};

// IMPLEMENTATION //////////////////////////////

template <class ComparatorTyp, class DataTyp>
GenericBufferedComparator<ComparatorTyp, DataTyp>::GenericBufferedComparator(const ComparatorTyp & comparator):
myComparator(comparator),
myTotalData(0)
{}

template <class ComparatorTyp, class DataTyp>
double GenericBufferedComparator<ComparatorTyp, DataTyp>::compareStructures(
  const IBufferedComparator::DataHandle & h1,
  const IBufferedComparator::DataHandle & h2) const
{
  const typename DataMap::const_iterator end = myDataMap.end();

  // Find comparison data 1
  typename DataMap::const_iterator it = myDataMap.find(h1);
  if(it == end)
  {
    // TODO: Throw
  }
  const DataTyp & compDat1 = *it->second;

  // Find comparison data 2
  it = myDataMap.find(h2);
  if(it == end)
  {
    // TODO: Throw
  }
  const DataTyp & compDat2 = *it->second;

  return myComparator.compareStructures(compDat1, compDat2);
}

template <class ComparatorTyp, class DataTyp>
bool GenericBufferedComparator<ComparatorTyp, DataTyp>::areSimilar(
  const IBufferedComparator::DataHandle & h1,
  const IBufferedComparator::DataHandle & h2) const
{
  const typename DataMap::const_iterator end = myDataMap.end();

  // Find comparison data 1
  typename DataMap::const_iterator it = myDataMap.find(h1);
  if(it == end)
  {
    // TODO: Throw
  }
  const DataTyp & compDat1 = *it->second;

  // Find comparison data 2
  it = myDataMap.find(h2);
  if(it == end)
  {
    // TODO: Throw
  }
  const DataTyp & compDat2 = *it->second;

  return myComparator.areSimilar(compDat1, compDat2);
}

template <class ComparatorTyp, class DataTyp>
const IBufferedComparator::DataHandle
GenericBufferedComparator<ComparatorTyp, DataTyp>::generateComparisonData(const ::sstbx::common::Structure & str)
{
  ::boost::shared_ptr<const DataTyp> data(myComparator.generateComparisonData(str));
  myDataMap.insert(typename DataMap::value_type(++myTotalData, data));
  return myTotalData;
}

template <class ComparatorTyp, class DataTyp>
void
GenericBufferedComparator<ComparatorTyp, DataTyp>::releaseComparisonData(IBufferedComparator::DataHandle & handle)
{
  // Find comparison data
  const typename DataMap::iterator it = myDataMap.find(handle);
  if(it == myDataMap.end())
  {
    // TODO: Throw
  }
  myDataMap.erase(it);
  handle = UNINITIALISED;
}

template <class ComparatorTyp, class DataTyp>
const ComparatorTyp & GenericBufferedComparator<ComparatorTyp, DataTyp>::getComparator() const
{
  return myComparator;
}

}
}

#endif /* GENERIC_BUFFERED_COMPARATOR_H */
