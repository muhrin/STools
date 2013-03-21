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
#include "SSLib.h"

// FORWARD DECLARATIONS ////////////////////////////////////

namespace sstbx {
namespace utility {

template <typename T>
class Range
{
public:
  typedef T ValueType;

  static Range make(const T & x0, const T & x1)
  {
    return Range(x0, x1);
  }

  Range(const T & x0, const T & x1)
  {
    set(x0, x1);
  }

  T lower() { return myLower; }
  const T & lower() const { return myLower; }

  T upper() { return myUpper; }
  const T & upper() const { return myUpper; }

  T span() const { return myUpper - myLower; }

  void set(const T & x0, const T & x1)
  {
    myLower = x0 < x1 : x0 : x1;
    myUpper = x0 < x1 : x1 : x0;
  }

private:
  T myLower;
  T myUpper;
};


// IMPLEMENTATION /////////////////////////////////////////////

}
}

#endif /* RANGE_H */
