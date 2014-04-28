/*
 * LoadStructuresTest.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "spipetest.h"

#include <pipelib/pipelib.h>

#include <spl/common/AtomSpeciesDatabase.h>
#include <spl/common/Structure.h>
#include <spl/io/IoFunctions.h>

// From SPipe
#include <SpTypes.h>
#include <StructurePipe.h>
#include <common/SharedData.h>
#include <common/StructureData.h>
#include <blocks/LoadStructures.h>

namespace ssc = spl::common;
namespace blocks = spipe::blocks;
namespace fs = boost::filesystem;

class LoadStructureChecker : public spipe::FinishedSink
{
  typedef spipe::StructureDataUniquePtr StructureDataPtr;

public:
  LoadStructureChecker(const std::vector< fs::path> & structures) :
      myStructurePaths(structures.begin(), structures.end())
  {
  }

  virtual void
  finished(StructureDataPtr data)
  {
    const spl::io::ResourceLocator * const loc =
        data->getStructure()->getProperty(
            spl::common::structure_properties::io::LAST_ABS_FILE_PATH);
    BOOST_REQUIRE(loc);

    const std::set< fs::path>::const_iterator it = myStructurePaths.find(
        loc->string());
    BOOST_REQUIRE(it != myStructurePaths.end());

    myStructurePaths.erase(it);
  }

  bool
  passed()
  {
    return myStructurePaths.empty();
  }

private:
  std::set< fs::path> myStructurePaths;
};

BOOST_AUTO_TEST_CASE(LoadStructuresTest)
{
  typedef spipe::SerialEngine Engine;

  static const std::string WILDCARD_PATHS = "structures/*.res";

  // Get the paths to all the files we're going to test
  std::vector< boost::filesystem::path> structures;
  spl::io::getWildcardPaths(WILDCARD_PATHS, structures);

  LoadStructureChecker sink(structures);

  spipe::BlockHandle load(new blocks::LoadStructures("structures/*.res"));

  Engine engine;
  engine.setFinishedDataSink(&sink);
  engine.run(load);

  BOOST_REQUIRE(sink.passed());
}

