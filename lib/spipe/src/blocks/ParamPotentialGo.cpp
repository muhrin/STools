/*
 * ParamPotentialGo.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "blocks/ParamPotentialGo.h"

#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>

#include <pipelib/IPipeline.h>

// From SSTbx
#include <common/Structure.h>
#include <potential/StandardData.h>
#include <potential/IGeomOptimiser.h>
#include <potential/IParameterisable.h>

#include "common/PipeFunctions.h"
#include "common/StructureData.h"
#include "common/SharedData.h"
#include "common/UtilityFunctions.h"

// NAMESPACES ////////////////////////////////


namespace spipe
{
namespace blocks
{

namespace fs = ::boost::filesystem;
namespace common = ::spipe::common;
namespace utility = ::spipe::utility;

ParamPotentialGo::ParamPotentialGo(
	::sstbx::potential::IParameterisable & paramPotential,
	::sstbx::potential::IGeomOptimiser & optimiser):
pipelib::Block<StructureDataTyp, SharedDataTyp>("Parameterised potential geometry optimisation"),
myParamPotential(paramPotential),
myOptimiser(optimiser),
myTableSupport(fs::path("param_pot.dat"))
{}

void ParamPotentialGo::pipelineInitialising()
{
  myTableSupport.registerPipeline(*myPipeline);
}

void ParamPotentialGo::pipelineStarting()
{
  // The pipeline is starting so try and get the potential parameters
  common::ObjectData<arma::vec>
    params = common::getObject(common::GlobalKeys::POTENTIAL_PARAMS, *myPipeline);

  if(params.first != common::DataLocation::NONE)
  {
    // Need to get the actual parameters as the potential may use a combining rule or change
    // them in some way from those specified
    myCurrentParams = setPotentialParams(*params.second);

    // The potential may have changed the params so reset them in the shared data
    common::setObject(
      common::GlobalKeys::POTENTIAL_PARAMS,
      params.first,
      myCurrentParams,
      *myPipeline
    );

    // Add a note to the table with the current parameter string
    myTableSupport.getTable().addTableNote("params: " + myParamPotential.getParamString());
  }
}

void ParamPotentialGo::in(spipe::common::StructureData & data)
{
  ::boost::shared_ptr<sstbx::potential::StandardData<> > optData;
	if(myOptimiser.optimise(*data.getStructure(), optData))
  {
	  // Copy over information from the optimisation results
	  data.enthalpy.reset(optData->totalEnthalpy);
	  data.stressMtx.reset(optData->stressMtx);

    data.objectsStore.insert(common::GlobalKeys::POTENTIAL_PARAMS, myCurrentParams);

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

arma::vec ParamPotentialGo::setPotentialParams(const ::arma::vec & params)
{
	using ::std::string;

	myParamPotential.setParams(params);

  const arma::vec actualParams = myParamPotential.getParams();

  return actualParams;
}

void ParamPotentialGo::updateTable(const ::spipe::StructureDataTyp & strData)
{
  utility::DataTable & table = myTableSupport.getTable();
  const ::std::string & strName = *strData.name;

  if(strData.enthalpy)
    table.insert(strName, "energy", ::boost::lexical_cast< ::std::string>(*strData.enthalpy));
}


}}

