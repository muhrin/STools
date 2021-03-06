/*
 * CastepGeomOptimiserTest.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "sslibtest.h"

#include <SSLib.h>
#include <common/AtomSpeciesDatabase.h>
#include <common/Structure.h>
#include <io/CastepReader.h>
#include <io/CellReaderWriter.h>
#include <potential/CastepGeomOptimiser.h>
#include <potential/OptimisationSettings.h>
#include <utility/StableComparison.h>

namespace ssc = ::sstbx::common;
namespace ssio = ::sstbx::io;
namespace ssp = ::sstbx::potential;
namespace ssu = ::sstbx::utility;

void testCastepOutput(ssc::Structure structure, const ::std::string & seed, const double enthalpy, const double pressure);

BOOST_AUTO_TEST_CASE(ReadCastepOutputTest)
{
  // Settings
  const size_t NUM_FILES = 2;
  static const ::std::string SEEDS[] = {"run1", "run2"};
  static const double ENTHALPIES[] = {-1.79788727e2, -1924.275193};
  static const double PRESSURES[] = {-0.0081437, 100.040623};

  for(size_t i = 0; i < NUM_FILES; ++i)
  {
    ssc::Structure structure;
    testCastepOutput(structure, SEEDS[i], ENTHALPIES[i], PRESSURES[i]);
  }
  // Now test that it also works if you update a pre-existing structure
  ssc::Structure structure;
  for(size_t i = 0; i < NUM_FILES; ++i)
  {
    testCastepOutput(structure, SEEDS[i], ENTHALPIES[i], PRESSURES[i]);
  }
}

void testCastepOutput(ssc::Structure structure, const ::std::string & seed, const double enthalpy, const double pressure)
{
  static const ssc::AtomSpeciesDatabase SPECIES_DB;
  static const ssio::CastepReader CASTEP_READER;
  static const ssio::CellReaderWriter CELL_READER;

  ssp::OptimisationSettings optSettings;

  ssp::CastepGeomOptimiseSettings runSettings;
  // Keep intermediates so we don't delete the input files
  runSettings.keepIntermediateFiles = true;

  ssp::detail::CastepGeomOptRun optRun(
    optSettings,
    seed,
    seed,
    CELL_READER,
    CASTEP_READER,
    runSettings
  );
  ssp::OptimisationData optimisationData;
  const bool updatedSuccessfully = optRun.updateStructure(structure, optimisationData, SPECIES_DB).isSuccess();
  BOOST_REQUIRE(updatedSuccessfully);

  // Enthalpy
  BOOST_REQUIRE(optimisationData.enthalpy);
  BOOST_REQUIRE(ssu::StableComp::eq(*optimisationData.enthalpy, enthalpy));

  const double * strEnthalpy = structure.getProperty(ssc::structure_properties::general::ENTHALPY);
  BOOST_REQUIRE(strEnthalpy && ssu::StableComp::eq(*strEnthalpy, enthalpy));

  // Pressure
  BOOST_REQUIRE(optimisationData.pressure);
  BOOST_REQUIRE(ssu::StableComp::eq(*optimisationData.pressure, pressure, 1e-4));

  const double * strPressure = structure.getProperty(ssc::structure_properties::general::PRESSURE_INTERNAL);
  BOOST_REQUIRE(strPressure && ssu::StableComp::eq(*strPressure, pressure));
}
