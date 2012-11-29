/*
 * PotentialGo.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "PotentialGo.h"

#include <locale>
#include <set>

#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/optional.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/tokenizer.hpp>

#include <pipelib/IPipeline.h>

// From SSTbx
#include <build_cell/AtomsDescription.h>
#include <build_cell/StructureDescription.h>
#include <build_cell/StructureDescriptionVisitor.h>
#include <common/Structure.h>
#include <potential/PotentialData.h>
#include <potential/IGeomOptimiser.h>
#include <potential/IPotential.h>

#include "common/StructureData.h"
#include "common/SharedData.h"
#include "common/UtilityFunctions.h"

// NAMESPACES ////////////////////////////////


namespace spipe {
namespace blocks {

namespace ssbc = ::sstbx::build_cell;
namespace ssc = ::sstbx::common;
namespace ssp = ::sstbx::potential;
namespace structure_properties = ssc::structure_properties;

PotentialGo::PotentialGo(
	const sstbx::potential::IGeomOptimiser & optimiser,
  const ::arma::mat33 * const              externalPressure,
  const bool                               writeOutput):
pipelib::Block<StructureDataTyp, SharedDataTyp>("Potential geometry optimisation"),
myOptimiser(optimiser),
myWriteOutput(writeOutput)
{
  if(externalPressure)
  {
    myExternalPressure = *externalPressure;
  }
  else
  {
    myExternalPressure.zeros();
  }
}

void PotentialGo::pipelineInitialising()
{
  if(myWriteOutput)
  {
    myTableSupport.setFilename(myPipeline->getGlobalData().getOutputFileStem().string() + ".geomopt");
  }
  myTableSupport.registerPipeline(*myPipeline);
}

void PotentialGo::in(spipe::common::StructureData & data)
{
  ssp::PotentialData optData;
  ssc::Structure * const structure = data.getStructure();
	if(myOptimiser.optimise(*structure, optData, myExternalPressure))
  {
    // Copy over information from the optimisation results
    copyOptimisationResults(optData, *structure);

    // Update our data table with the structure data
    updateTable(*structure);

	  myOutput->in(data);
  }
  else
  {
    // The structure failed to geometry optimise properly so drop it
    myPipeline->dropData(data);
  }
}

void PotentialGo::copyOptimisationResults(
  const sstbx::potential::PotentialData & optData,
  ssc::Structure & structure)
{
  // Copy over information from the optimisation results

  // Internal energy
  structure.setProperty(
    structure_properties::general::ENERGY_INTERNAL,
    optData.internalEnergy
  );

  // Pressure
  const double pressure = ::arma::trace(optData.stressMtx) / 3.0;
  structure.setProperty(
    structure_properties::general::ENERGY_INTERNAL,
    pressure
  );
}

void PotentialGo::updateTable(const ssc::Structure & structure)
{
  utility::DataTable & table = myTableSupport.getTable();
  const ::std::string & strName = structure.getName();

  const double * const internalEnergy = structure.getProperty(structure_properties::general::ENERGY_INTERNAL);
  if(internalEnergy)
  {
    table.insert(strName, "energy", common::getString(*internalEnergy));
    table.insert(strName, "energy/atom", common::getString(*internalEnergy / structure.getNumAtoms()));
  }
}

}
}

