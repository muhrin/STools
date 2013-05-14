/*
 * LandscapeExplorerOptimiser.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "potential/LandscapeExplorerOptimiser.h"

#include "potential/IControllableOptimiser.h"
#include "utility/IStructureComparator.h"

// NAMESPACES ////////////////////////////////
namespace sstbx {
namespace potential {

LandscapeExplorerOptimiser::LandscapeExplorerOptimiser(
  OptimiserPtr optimiser,
  ComparatorPtr comparator
):
myExplorer(comparator, true),
myOptimiser(optimiser)
{
  myOptimiser->setController(myExplorer);
}

IPotential * LandscapeExplorerOptimiser::getPotential()
{
  return myOptimiser->getPotential();
}

const IPotential * LandscapeExplorerOptimiser::getPotential() const
{
  return myOptimiser->getPotential();
}

OptimisationOutcome LandscapeExplorerOptimiser::optimise(
  common::Structure & structure,
  const OptimisationSettings & options
) const
{
  return myOptimiser->optimise(structure, options);
}

OptimisationOutcome LandscapeExplorerOptimiser::optimise(
	common::Structure & structure,
  OptimisationData & data,
  const OptimisationSettings & options
) const
{
  return myOptimiser->optimise(structure, data, options);
}

}
}

