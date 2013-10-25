/*
 * Range.h
 *
 * Tuple that represents a range.
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef RANGE_H
#define RANGE_H

// INCLUDES /////////////////////////////////////////////
#include "spl/SSLib.h"

#include <boost/concept/assert.hpp>
#include <boost/concept_check.hpp>

// FORWARD DECLARATIONS ////////////////////////////////////

namespace spl {
namespace utility {

template <typename T>
class MinMax
{
public:
  BOOST_CONCEPT_ASSERT((::boost::LessThanComparable<T>));

  typedef T ValueType;

  static MinMax make(const T & x0, const T & x1)
  {
    return MinMax(x0, x1);
  }

  MinMax() {}

  explicit MinMax(const T & x)
  {
    set(x, x);
  }

  MinMax(const T & x0, const T & x1)
  {
    set(x0, x1);
  }

  MinMax(const MinMax & toCopy)
  {
    myLower = toCopy.myLower;
    myUpper = toCopy.myUpper;
  }

  MinMax & operator =(const MinMax & rhs)
  {
    myLower = rhs.myLower;
    myUpper = rhs.myUpper;
    return *this;
  }

  T lower() { return myLower; }
  const T & lower() const { return myLower; }

  T upper() { return myUpper; }
  const T & upper() const { return myUpper; }

  void set(const T & x0, const T & x1)
  {
    myLower = x0 < x1 ? x0 : x1;
    myUpper = x0 < x1 ? x1 : x0;
  }

  bool operator < (const MinMax & rhs) const
  {
    if(myLower < rhs.myLower)
      return true;
    if(rhs.myLower < myLower)
      return false;

    if(myUpper < rhs.myUpper)
      return true;
    return false;
  }

protected:
  T myLower;
  T myUpper;
};

template <typename T>
class Range : public MinMax<T>
{
public:
  BOOST_CONCEPT_ASSERT((::boost::LessThanComparable<T>));

  typedef T ValueType;

  static Range make(const T & x0, const T & x1)
  {
    return Range(x0, x1);
  }

  Range() {}

  explicit Range(const T & x): MinMax<T>(x, x)
  {
  }

  Range(const T & x0, const T & x1): MinMax<T>(x0, x1)
  {
  }

  bool nullSpan() const
  { return MinMax<T>::myLower == MinMax<T>::myUpper; }


  T span() const { return MinMax<T>::myUpper - MinMax<T>::myLower; }
};


// IMPLEMENTATION /////////////////////////////////////////////

}
}

#endif /* RANGE_H */
