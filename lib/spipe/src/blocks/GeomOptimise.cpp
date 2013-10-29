/*
 * GeomOptimise.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "blocks/GeomOptimise.h"

#include <boost/lexical_cast.hpp>

#include <spl/build_cell/AtomsDescription.h>
#include <spl/common/Structure.h>
#include <spl/potential/PotentialData.h>
#include <spl/potential/IGeomOptimiser.h>
#include <spl/potential/IPotential.h>

#include "common/PipeFunctions.h"
#include "common/StructureData.h"
#include "common/SharedData.h"
#include "common/UtilityFunctions.h"

// NAMESPACES ////////////////////////////////

namespace spipe {
namespace blocks {

namespace ssbc = ::spl::build_cell;
namespace ssc = ::spl::common;
namespace ssp = ::spl::potential;
namespace structure_properties = ssc::structure_properties;

GeomOptimise::GeomOptimise(spl::potential::IGeomOptimiserPtr optimiser,
    const bool writeSummary) :
    Block("Geometry optimise"), myWriteSummary(writeSummary), myOptimisationParams(), myOptimiser(
        optimiser)
{
}

GeomOptimise::GeomOptimise(spl::potential::IGeomOptimiserPtr optimiser,
    const ::spl::potential::OptimisationSettings & optimisationParams,
    const bool writeSummary) :
    Block("Potential geometry optimisation"), myWriteSummary(writeSummary), myOptimisationParams(
        optimisationParams), myOptimiser(optimiser)
{
}

void
GeomOptimise::pipelineInitialising()
{
  if(myWriteSummary)
  {
    myTableSupport.setFilename(
        common::getOutputFileStem(getEngine()->sharedData(),
            getEngine()->globalData()) + ".geomopt");
    myTableSupport.registerEngine(getEngine());
  }
}

void
GeomOptimise::in(common::StructureData * const data)
{
  ssc::Structure * const structure = data->getStructure();
  ssp::OptimisationData optData;
  const ssp::OptimisationOutcome outcome = myOptimiser->optimise(*structure,
      optData, myOptimisationParams);
  if(outcome.isSuccess())
  {
    // Update our data table with the structure data
    if(myWriteSummary)
      updateTable(*structure, optData);
    out(data);
  }
  else
  {
    ::std::cerr << "Optimisation failed: " << outcome.getMessage()
        << ::std::endl;
    // The structure failed to geometry optimise properly so drop it
    drop(data);
  }
}

ssp::IGeomOptimiser &
GeomOptimise::getOptimiser()
{
  return *myOptimiser;
}

::spipe::utility::DataTableSupport &
GeomOptimise::getTableSupport()
{
  return myTableSupport;
}

void
GeomOptimise::updateTable(const ssc::Structure & structure,
    const ::spl::potential::OptimisationData & optimisationData)
{
  utility::DataTable & table = myTableSupport.getTable();
  const ::std::string & strName = structure.getName();

  const double * const internalEnergy = structure.getProperty(
      structure_properties::general::ENERGY_INTERNAL);
  if(internalEnergy)
  {
    table.insert(strName, "energy", common::getString(*internalEnergy));
    table.insert(strName, "energy/atom",
        common::getString(
            *internalEnergy / static_cast< double>(structure.getNumAtoms())));
    if(optimisationData.numIters)
      table.insert(strName, "iters",
          ::boost::lexical_cast< ::std::string>(*optimisationData.numIters));
  }
}

}
}

