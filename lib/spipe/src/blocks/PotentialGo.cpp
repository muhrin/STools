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


namespace spipe
{
namespace blocks
{

namespace ssbc = ::sstbx::build_cell;
namespace ssc = ::sstbx::common;
namespace ssp = ::sstbx::potential;

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
    myExternalPressure.fill(0.0);
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
	if(myOptimiser.optimise(*data.getStructure(), optData, myExternalPressure))
  {
    // Copy over information from the optimisation results
    copyOptimisationResults(optData, data);

    // Update our data table with the structure data
    updateTable(data);

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
  spipe::common::StructureData & strData)
{
  // Copy over information from the optimisation results

  // Enthaly
  strData.enthalpy.reset(optData.internalEnergy);

  // Pressure
  const double pressure = ::arma::trace(optData.stressMtx) / 3.0;
  strData.pressure.reset(pressure);
  strData.objectsStore.insert(common::StructureObjectKeys::PRESSURE_INTERNAL, pressure); 
}

void PotentialGo::updateTable(const ::spipe::StructureDataTyp & strData)
{
  utility::DataTable & table = myTableSupport.getTable();
  const ::std::string & strName = *strData.name;

  if(strData.enthalpy)
    table.insert(strName, "energy", ::boost::lexical_cast< ::std::string>(*strData.enthalpy));
}

}
}

