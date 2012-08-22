/*
 * MultiIdxRange.h
 *
 * Range in multidimensional index space.
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef MULTI_IDX_RANGE_H
#define MULTI_IDX_RANGE_H

// INCLUDES /////////////////////////////////////////////
#include "SSLib.h"

#include <boost/iterator/iterator_facade.hpp>
#include <boost/range/iterator_range.hpp>

#include "utility/MultiIdx.h"

// FORWARD DECLARATIONS ////////////////////////////////////

namespace sstbx {
namespace utility {

template <typename Integer>
class MultiIdxRange;

namespace utility_detail {

template <typename Integer>
class ConstMultiIdxIterator : public ::boost::iterator_facade<
  ConstMultiIdxIterator<Integer>,
  const MultiIdx<Integer>,
  ::boost::random_access_traversal_tag,
  const MultiIdx<Integer> &,
  MultiIdx<Integer>
>
{
  typedef ::boost::iterator_facade<
    ConstMultiIdxIterator<Integer>,
    const MultiIdx<Integer>,
    ::boost::random_access_traversal_tag,
    const MultiIdx<Integer> &,
    MultiIdx<Integer>
  > base_t;

public:
  typedef typename base_t::value_type value_type;
  typedef typename base_t::difference_type difference_type;
  typedef typename base_t::reference reference;

  explicit ConstMultiIdxIterator(value_type x, const MultiIdxRange<Integer> & range) : myValue(x), myRange(range) {}

private:

  void increment()
  {
	  bool increment = true;
	  for(size_t i = 0; increment && i < myRange.dims(); ++i)
	  {
		  // Increment the position vector and check it does not exceed
		  // the max number of loops
		  if(++myValue[i] >= myRange.myLast[i])
		  {
			  myValue[i] = 0;
		  }
		  else
		  {
			  // Successfully moved one position on in index space
			  increment = false;
		  }
	  }

    // Do we still need to increment even though we have covered all digits?
    if(increment)
    {
      // Then we have reached the end
      myValue = myRange.myLast;
    }
  }

  void decrement()
  {
    // TODO:!!!
	  //bool decrement = true;
	  //for(size_t i = 0; decrement && i > myRange.dims(); ++i)
	  //{
		 // // Increment the position vector and check it does not exceed
		 // // the max number of loops
		 // if(++myValue[i] <= myRange.myLast[i])
		 // {
			//  myValue[i] = 0;
		 // }
		 // else
		 // {
			//  // Successfully moved one position on in index space
			//  increment = false;
		 // }
	  //}
  }

  void advance(difference_type & offset)
  {
    myValue += offset;
  }

  difference_type distance_to(const ConstMultiIdxIterator & other) const
  {
    return other.myValue - myValue;
  }

  bool equal(const ConstMultiIdxIterator & other) const
  {
    return myValue == other.myValue;
  }

  reference dereference() const
  {
    return myValue;
  }

  value_type                      myValue;
  const MultiIdxRange<Integer> &  myRange;


  friend class ::boost::iterator_core_access;
};

} // namespace utility_details

template <typename Integer>
class MultiIdxRange : public ::boost::iterator_range< utility_detail::ConstMultiIdxIterator<Integer> >
{
  typedef utility_detail::ConstMultiIdxIterator<Integer> iterator_t;
  typedef ::boost::iterator_range<iterator_t> base_t;

public:
  MultiIdxRange(const MultiIdx<Integer> & first, const MultiIdx<Integer> & last):
    myFirst(first),
    myLast(last),
    base_t(iterator_t(first, *this), iterator_t(last, *this))
  {
    SSE_ASSERT(myFirst <= myLast);
  }

  size_t dims() const
  {
    return myFirst.dims();
  }

private:


  const MultiIdx<Integer> myFirst;
  const MultiIdx<Integer> myLast;

  // Sadly can't use iterator_t typedef here as this behaviour is not supported
  // until C++11.
  // See e.g.: http://stackoverflow.com/questions/392120/why-cant-i-declare-a-friend-through-a-typedef
  friend class utility_detail::ConstMultiIdxIterator<Integer>;
};


} // namespace utility
} // namespace sslib

#endif /* MULTI_IDX_RANGE_H */
