/*
 * GeomOptimise.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "blocks/GeomOptimise.h"

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

GeomOptimise::GeomOptimise(spl::potential::IGeomOptimiserPtr optimiser) :
    Block("Geometry optimise"), myOptimiser(optimiser), myWriteOutput(
        true), myOptimisationParams()
{
}

GeomOptimise::GeomOptimise(spl::potential::IGeomOptimiserPtr optimiser,
    const ::spl::potential::OptimisationSettings & optimisationParams) :
    Block("Potential geometry optimisation"), myOptimiser(optimiser), myWriteOutput(
        true), myOptimisationParams(optimisationParams)
{
}

void
GeomOptimise::pipelineInitialising()
{
  if(myWriteOutput)
    myTableSupport.setFilename(
        common::getOutputFileStem(getEngine()->sharedData(), getEngine()->globalData()) + ".geomopt");
  myTableSupport.registerEngine(getEngine());
}

void
GeomOptimise::in(common::StructureData * const data)
{
  ssc::Structure * const structure = data->getStructure();
  const ssp::OptimisationOutcome outcome = myOptimiser->optimise(*structure,
      myOptimisationParams);
  if(outcome.isSuccess())
  {
    // Update our data table with the structure data
    updateTable(*structure);
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

bool
GeomOptimise::getWriteOutput() const
{
  return myWriteOutput;
}

void
GeomOptimise::setWriteOutput(const bool write)
{
  myWriteOutput = write;
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
GeomOptimise::updateTable(const ssc::Structure & structure)
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
  }
}

}
}

