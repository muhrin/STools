/*
 * DistanceCalculatorsTest.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "sslibtest.h"

#include <boost/foreach.hpp>

#include <io/CastepReader.h>
#include <io/CellReaderWriter.h>
#include <potential/CastepRun.h>

// NAMESPACES ///////////////////////////////
namespace ssio = ::sstbx::io;
namespace ssp = ::sstbx::potential;

BOOST_AUTO_TEST_CASE(CastepRunTest)
{
  ::std::vector< ::std::string> SEEDS;
  SEEDS.push_back("run1");
  SEEDS.push_back("run2");
  SEEDS.push_back("run3");
  SEEDS.push_back("run4");

  ssio::CastepReader castepReader;
  ssio::CellReaderWriter cellReader;

  for(size_t i = 0; i < 3; ++i)
  {
    ssp::CastepRun castepRun(SEEDS[i], cellReader, castepReader);
    BOOST_REQUIRE(castepRun.finishedSuccessfully());
  }
  for(size_t i = 3; i < 4; ++i)
  {
    ssp::CastepRun castepRun(SEEDS[i], cellReader, castepReader);
    BOOST_REQUIRE(!castepRun.finishedSuccessfully());
  }
}
