/*
 * PasteFragmentTest.cpp
 *
 *  Created on: Feb 5, 2014
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "spipetest.h"

#include <string>

#include <boost/foreach.hpp>

#include <armadillo>

#include <spl/common/Structure.h>
#include <spl/math/Matrix.h>
#include <spl/math/Random.h>

// From SPipe
#include <SpTypes.h>
#include <StructurePipe.h>
#include <common/SharedData.h>
#include <common/StructureData.h>
#include <blocks/PasteFragment.h>

using namespace spipe;

namespace splc = spl::common;
namespace splm = spl::math;
namespace blocks = spipe::blocks;

BOOST_AUTO_TEST_SUITE(PasteFragmentTest)

class SeedBlock : public StartBlock
{
public:
  SeedBlock(const splc::Structure & structure) :
      Block("Seed block")
  {
    myStructure = structure;
  }
  virtual void
  start()
  {
    common::StructureData * const strData = getEngine()->createData();
    strData->setStructure(myStructure.clone());
    out(strData);
  }
private:
  splc::Structure myStructure;
};

class PasteTester : public FinishedSink
{
  typedef spipe::StructureDataUniquePtr StructureDataPtr;
public:

  PasteTester(const splc::Structure & structure,
      const splc::Structure & fragment, const arma::mat44 & transform);

  // This is where the check should actually happen
  virtual void
  finished(StructureDataPtr data);

private:
  void
  checkEqual(const arma::vec3 & a, const arma::vec3 & b) const;

  splc::Structure myOriginalStructure;
  splc::Structure myPastedFragment;
  arma::mat44 myFragmentTransform;
};

PasteTester::PasteTester(const splc::Structure & structure,
    const splc::Structure & fragment, const arma::mat44 & transform) :
    myOriginalStructure(structure), myPastedFragment(fragment), myFragmentTransform(
        transform)
{
}

void
PasteTester::finished(StructureDataPtr data)
{
  BOOST_REQUIRE(data->getStructure());
  const splc::Structure & structure = *data->getStructure();

  const size_t totalAtoms = myOriginalStructure.getNumAtoms()
      + myPastedFragment.getNumAtoms();
  BOOST_REQUIRE_EQUAL(structure.getNumAtoms(), totalAtoms);

  // Now check all the atoms are as expected
  arma::vec3 origPos, newPos;
  for(size_t i = 0; i < myOriginalStructure.getNumAtoms(); ++i)
  {
    origPos = myOriginalStructure.getAtom(i).getPosition();
    newPos = structure.getAtom(i).getPosition();
    checkEqual(origPos, newPos);
  }
  // Now check the 'pasted atoms'
  for(size_t i = 0; i < myPastedFragment.getNumAtoms(); ++i)
  {
    origPos = spl::math::transformCopy(
        myPastedFragment.getAtom(i).getPosition(), myFragmentTransform);
    newPos = structure.getAtom(i + myOriginalStructure.getNumAtoms()).getPosition();
    checkEqual(origPos, newPos);
  }
}

void
PasteTester::checkEqual(const arma::vec3 & a, const arma::vec3 & b) const
{
  for(size_t i = 0; i < 3; ++i)
    BOOST_CHECK_CLOSE_FRACTION(a(i), b(i), 0.0001);
}

BOOST_AUTO_TEST_CASE(DirectStructurePaste)
{
  typedef SerialEngine Engine;

  // SETTINGS //
  static const size_t NUM_STRUCTURE_ATOMS = 10;
  static const size_t NUM_FRAGMENT_ATOMS = 5;

  // Create the fragment to paste
  std::string species;
  splc::Structure fragment;
  for(size_t i = 0; i < NUM_FRAGMENT_ATOMS; ++i)
  {
    species = splm::randu('A', 'Z');
    fragment.newAtom(species).setPosition(arma::randu(3));
  }

  // Create the initial structure to receive the fragment
  splc::Structure structure;
  for(size_t i = 0; i < NUM_STRUCTURE_ATOMS; ++i)
  {
    splm::randu('A', 'Z');
    structure.newAtom(species).setPosition(arma::randu(3));
  }

  // Create a random transform for the fragment
  arma::mat44 transform;
  splm::setTranslation(transform, arma::randu(3));
  splm::setRotation(transform, arma::randu(4));

  BlockHandle paste(new spipe::blocks::PasteFragment(fragment, transform));
  BlockHandle seed(new SeedBlock(structure));
  seed->connect(paste);

  PasteTester tester(structure, fragment, transform);
  Engine engine;
  engine.setFinishedDataSink(&tester);
  engine.run(seed);
}

BOOST_AUTO_TEST_SUITE_END()
