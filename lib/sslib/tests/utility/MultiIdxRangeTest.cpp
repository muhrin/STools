/*
 * MultiIdxRangeTest.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include <iostream>

#include <boost/foreach.hpp>

#include <utility/MultiIdx.h>
#include <utility/MultiIdxRange.h>

#include <boost/test/unit_test.hpp>

namespace ssu = ::sstbx::utility;

BOOST_AUTO_TEST_CASE(SimpleRange)
{
  ssu::MultiIdx<unsigned int> x0(2), x1(2);
  
  x0.fill(0); // Place start at 0,0
  x1.fill(3); // Place end at 3,3

  ssu::MultiIdxRange<unsigned int> range(x0, x1);

  size_t i = 0;
  for(ssu::MultiIdxRange<unsigned int>::const_iterator it = range.begin(), end = range.end();
    it != end; ++it)
  {
    BOOST_CHECK(i % 3 == (*it)[0]);
    ++i;
  }
  BOOST_CHECK(i == (x1 - x0).product());
}

BOOST_AUTO_TEST_CASE(OneDimensionalRange)
{
  ssu::MultiIdx<unsigned int> x0(1), x1(1);
  
  x0.fill(0); // Place start at 0
  x1.fill(41); // Place end at 41

  const ssu::MultiIdxRange<unsigned int> range(x0, x1);

  size_t i = 0;
  for(ssu::MultiIdxRange<unsigned int>::const_iterator it = range.begin(), end = range.end();
    it != end; ++it)
  {
    // Make sure each entry is correct
    BOOST_CHECK(i == (*it)[0]);
    ++i;
  }
  BOOST_CHECK(i == (x1 - x0).product());
}
