/*
 * DistanceCalculatorsTest.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include <iostream>
#include <vector>

#include <boost/shared_ptr.hpp>
#include <boost/test/unit_test.hpp>

#include <armadillo>

#include <common/OrthoCellDistanceCalculator.h>
#include <common/Atom.h>
#include <common/Structure.h>
#include <common/Types.h>
#include <common/UniversalCrystalDistanceCalculator.h>
#include <common/UnitCell.h>
#include <utility/StableComparison.h>

namespace ssc = ::sstbx::common;
namespace ssu = ::sstbx::utility;

BOOST_AUTO_TEST_CASE(DistanceCalculatorsComparison)
{
  // SETTINGS ////////////////
  const size_t numAtoms = 50;
  const double cellDim = 10;
  const double tolerance = cellDim / 1e10;
  const double cutoffDist = 2.11 * cellDim;

  ssc::Structure structure;
  ssc::UnitCellPtr cell(new ssc::UnitCell(0.25 * cellDim, cellDim, 1.75 * cellDim, 90.0, 90.0, 90.0));

  structure.setUnitCell(cell);

  for(size_t i = 0; i < numAtoms; ++i)
    structure.newAtom(ssc::AtomSpeciesId::CUSTOM_1).setPosition(::arma::randu< ::arma::vec>(3));

  ssc::OrthoCellDistanceCalculator orthoCalc(structure);
  ssc::UniversalCrystalDistanceCalculator univCalc(structure);

  BOOST_REQUIRE(orthoCalc.isValid());
  BOOST_REQUIRE(univCalc.isValid());

  ::std::vector<double> orthoDist, univDist;

  // Timers
  time_t t0, t1;

  t0 = time(NULL);

  for(size_t i = 0; i < numAtoms; ++i)
  {
    const ssc::Atom atom1 = structure.getAtom(i);
    for(size_t j = i; j < numAtoms; ++j)
    {
      const ssc::Atom atom2 = structure.getAtom(j);
      univCalc.getDistsBetween(atom1, atom2, cutoffDist, univDist);
    }
  }

  t1 = time(NULL);

  ::std::cout << "Univ. took: " << t1 - t0 << ::std::endl;

  t0 = time(NULL);

  for(size_t i = 0; i < numAtoms; ++i)
  {
    const ssc::Atom atom1 = structure.getAtom(i);
    for(size_t j = i; j < numAtoms; ++j)
    {
      const ssc::Atom atom2 = structure.getAtom(j);
      orthoCalc.getDistsBetween(atom1, atom2, cutoffDist, orthoDist);
    }
  }

  t1 = time(NULL);

  ::std::cout << "Ortho took: " << t1 - t0 << ::std::endl;

  BOOST_REQUIRE(orthoDist.size() == univDist.size());

  ::std::sort(orthoDist.begin(), orthoDist.end());
  ::std::sort(univDist.begin(), univDist.end());

  const size_t numElements = ::std::min(orthoDist.size(), univDist.size());

  for(size_t i = 0; i < numElements; ++i)
  {
    BOOST_REQUIRE(ssu::StableComp::eq(orthoDist[i], univDist[i], tolerance));
  }
}


BOOST_AUTO_TEST_CASE(DistanceComparisonPathological)
{
  // SETTINGS ////////////////
  const size_t numAtoms = 4;
  const double cellDim = 3.1032302270780758;
  const double tolerance = cellDim / 1e16;
  const double cutoffDist = 5.00;

  ssc::Structure structure;
  //ssc::UnitCellPtr cell(new ssc::UnitCell(cellDim, cellDim, 3.1032310973902493, 89.999998840060258, 90.000001159939757, 89.999999999947718));
  ssc::UnitCellPtr cell(new ssc::UnitCell(cellDim, cellDim, 3.1032310973902493, 90.0, 90.0, 90.0));

  structure.setUnitCell(cell);

  // Put in four atoms
  structure.newAtom(ssc::AtomSpeciesId::NA).setPosition(-1.6581022716893747, -2.4672347213487633, -1.3431304885983484);
  structure.newAtom(ssc::AtomSpeciesId::NA).setPosition(-3.0839601555007641, -0.79937989707630863, -1.3574676571980899);
  structure.newAtom(ssc::AtomSpeciesId::NA).setPosition(-3.0355997292334425, -2.1869067011837431, 0.34178278728968065);
  structure.newAtom(ssc::AtomSpeciesId::NA).setPosition(-1.3834008907850099, -0.77711917814879217, 0.029117512324985273);


  ssc::OrthoCellDistanceCalculator orthoCalc(structure);
  ssc::UniversalCrystalDistanceCalculator univCalc(structure);

  BOOST_REQUIRE(orthoCalc.isValid());
  BOOST_REQUIRE(univCalc.isValid());

  ::std::vector<double> orthoDist, univDist;

  for(size_t i = 0; i < numAtoms; ++i)
  {
    const ssc::Atom atom1 = structure.getAtom(i);
    for(size_t j = i; j < numAtoms; ++j)
    {
      const ssc::Atom atom2 = structure.getAtom(j);
      univCalc.getDistsBetween(atom1, atom2, cutoffDist, univDist, 10000);
      orthoCalc.getDistsBetween(atom1, atom2, cutoffDist, orthoDist, 10000);
    }
  }

  BOOST_REQUIRE(orthoDist.size() == univDist.size());

  ::std::sort(orthoDist.begin(), orthoDist.end());
  ::std::sort(univDist.begin(), univDist.end());

  const size_t numElements = ::std::min(orthoDist.size(), univDist.size());

  for(size_t i = 0; i < numElements; ++i)
  {
    BOOST_REQUIRE(ssu::StableComp::eq(orthoDist[i], univDist[i], tolerance));
  }
}
