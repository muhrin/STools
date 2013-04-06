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
#include <utility/StableComparison.h>

namespace ssc = ::sstbx::common;
namespace ssio = ::sstbx::io;
namespace ssp = ::sstbx::potential;
namespace ssu = ::sstbx::utility;

BOOST_AUTO_TEST_CASE(ReadCastepOutputTest)
{
  // Settings
  static const double FINAL_ENTHALPY = -1.79788727e2;
  static const double FINAL_PRESSURE = -0.0081436;

  ssc::Structure structure;
  ssc::AtomSpeciesDatabase speciesDb;
  ssio::CastepReader castepReader;
  ssio::CellReaderWriter cellReader;
  ssp::detail::CastepGeomOptRun optRun("successful", "successful", false, cellReader, castepReader);
  ssp::OptimisationData optimisationData;
  const bool updatedSuccessfully = optRun.updateStructure(structure, optimisationData, speciesDb).isSuccess();
  BOOST_REQUIRE(updatedSuccessfully);

  // Enthalpy
  BOOST_REQUIRE(optimisationData.enthalpy);
  BOOST_REQUIRE(ssu::StableComp::eq(*optimisationData.enthalpy, FINAL_ENTHALPY));

  const double * strEnthalpy = structure.getProperty(ssc::structure_properties::general::ENTHALPY);
  BOOST_REQUIRE(strEnthalpy && ssu::StableComp::eq(*strEnthalpy, FINAL_ENTHALPY));

  // Pressure
  BOOST_REQUIRE(optimisationData.pressure);
  BOOST_REQUIRE(ssu::StableComp::eq(*optimisationData.pressure, FINAL_PRESSURE, 1e-5));

  const double * strPressure = structure.getProperty(ssc::structure_properties::general::PRESSURE_INTERNAL);
  BOOST_REQUIRE(strPressure && ssu::StableComp::eq(*strPressure, FINAL_PRESSURE));

}

