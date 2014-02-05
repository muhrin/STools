/*
 * CutAndPasteTest.cpp
 *
 *  Created on: Nov 26, 2013
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "spipetest.h"

#include <boost/ptr_container/ptr_vector.hpp>

#include <armadillo>

#include <spl/build_cell/BuildCellFwd.h>
#include <spl/build_cell/GenBox.h>
#include <spl/build_cell/GenCylinder.h>
#include <spl/build_cell/GenSphere.h>
#include <spl/common/Structure.h>
#include <spl/common/Types.h>
#include <spl/common/UnitCell.h>
#include <spl/math/Matrix.h>

#include <PipeLibTypes.h>
#include <blocks/BuildStructures.h>
#include <blocks/CutAndPaste.h>

using namespace spipe;

namespace splbc = spl::build_cell;
namespace splc = spl::common;
namespace ssc = spl::common;

typedef spl::UniquePtr< splbc::GeneratorShape>::Type GenShape;

BOOST_AUTO_TEST_SUITE(CutAndPasteTest)

static const size_t NUM_ATOMS = 100;

class SeedBlock : public StartBlock
{
public:
  SeedBlock(splc::StructurePtr structure) :
      Block("Seed block")
  {
    myStructure = structure;
  }
  virtual void
  start()
  {
    common::StructureData * const strData = getEngine()->createData();
    strData->setStructure(myStructure->clone());
    out(strData);
  }
private:
  splc::StructurePtr myStructure;
};

class CutChecker : public FinishedSink
{
  typedef spipe::StructureDataUniquePtr StructureDataPtr;
public:
  CutChecker(const splbc::GeneratorShape & genShape) :
      myShape(genShape)
  {
  }
  virtual
  void
  finished(StructureDataPtr data)
  {
    BOOST_REQUIRE(data->getStructure()->getNumAtoms() == 0);
  }
private:
  const splbc::GeneratorShape & myShape;
};

class PasteChecker : public FinishedSink
{
  typedef spipe::StructureDataUniquePtr StructureDataPtr;
public:
  PasteChecker(const splbc::GeneratorShape & genShape) :
      myShape(genShape)
  {
  }
  virtual
  void
  finished(StructureDataPtr data)
  {
    // Check we're left with the same number of atoms after pasting
    BOOST_REQUIRE(data->getStructure()->getNumAtoms() == NUM_ATOMS);
  }
private:
  const splbc::GeneratorShape & myShape;
};

void
createShapes(boost::ptr_vector< splbc::GeneratorShape> & shapes);

BOOST_AUTO_TEST_CASE(CutAndPasteTest)
{
  typedef SerialEngine Engine;

  static const splc::UnitCell UNIT_CELL(10.0, 10.0, 10.0, 90.0, 90.0, 90.0);

  // Create all the shapes that we want to check
  boost::ptr_vector< splbc::GeneratorShape> shapes;
  createShapes(shapes);

  blocks::CutAndPaste::Settings settings;
  // Check the cut functionality
  BOOST_FOREACH(splbc::GeneratorShape & shape, shapes)
  {
    splc::Structure structure;
    structure.setUnitCell(UNIT_CELL);

    for(size_t i = 0; i < NUM_ATOMS; ++i)
      structure.newAtom("Al").setPosition(shape.randomPoint());

    CutChecker sink(shape);

    BlockHandle seed(new SeedBlock(structure.clone()));
    BlockHandle cutAndPaste(new blocks::CutAndPaste(shape.clone(), settings));
    seed->connect(cutAndPaste);

    Engine engine;
    engine.setFinishedDataSink(&sink);
    engine.run(seed);
  }

  // Check cut and paste
  settings.paste = true;
  BOOST_FOREACH(splbc::GeneratorShape & shape, shapes)
  {
    splc::Structure structure;
    structure.setUnitCell(UNIT_CELL);

    for(size_t i = 0; i < NUM_ATOMS; ++i)
      structure.newAtom("Al").setPosition(shape.randomPoint());

    PasteChecker sink(shape);

    BlockHandle seed(new SeedBlock(structure.clone()));
    BlockHandle cutAndPaste(new blocks::CutAndPaste(shape.clone(), settings));
    seed->connect(cutAndPaste);

    Engine engine;
    engine.setFinishedDataSink(&sink);
    engine.run(seed);
  }
}

void
createShapes(boost::ptr_vector< splbc::GeneratorShape> & shapes)
{
  arma::vec3 centre;
  centre(0) = 5.0;
  centre(1) = 5.0;
  centre(2) = 5.0;

  shapes.push_back(
      new splbc::GenBox(4.0, 4.0, 4.0, spl::math::makeTranslation(centre)));

  {
    splbc::GenCylinder * const cylinder = new splbc::GenCylinder(5, 10);
    cylinder->setPosition(centre);
    shapes.push_back(cylinder);
  }

  {
    splbc::GenSphere * const sphere = new splbc::GenSphere(5);
    sphere->setPosition(centre);
    shapes.push_back(sphere);
  }

}

BOOST_AUTO_TEST_SUITE_END()
