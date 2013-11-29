/*
 * PointSeparator.cpp
 *
 *  Created on: Nov 28, 2013
 *      Author: Martin Uhrin
 */

#include "spl/build_cell/PointSeparator.h"

namespace spl {
namespace build_cell {

const int PointSeparator::DEFAULT_MAX_ITERATIONS = 1000;
const double PointSeparator::DEFAULT_TOLERANCE = 0.001;

SeparationData< common::AtomSpeciesId::Value>
makeSeparationData(const common::Structure & structure)
{
  const size_t numAtoms = structure.getNumAtoms();
  SeparationData< common::AtomSpeciesId::Value>
    options(numAtoms, structure.getDistanceCalculator());

  options.points.resize(numAtoms);
  for(size_t i = 0; i < numAtoms; ++i)
  {
    const common::Structure::StructureAtom & atom = structure.getAtom(i);
    options.points[i] = atom.getPosition();
    options.labels[i] = atom.getSpecies();
  }

  return options;
}

PointSeparator::PointSeparator() :
    myMaxIterations(DEFAULT_MAX_ITERATIONS), myTolerance(DEFAULT_TOLERANCE)
{
}

PointSeparator::PointSeparator(const size_t maxIterations,
    const double tolerance) :
    myMaxIterations(maxIterations), myTolerance(tolerance)
{
}

}
}
