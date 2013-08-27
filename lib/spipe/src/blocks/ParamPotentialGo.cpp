/*
 * ParamPotentialGo.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "blocks/ParamPotentialGo.h"

#include <sstream>

#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>

#include <spl/SSLibAssert.h> // TEMP !
#include <spl/common/Structure.h>
#include <spl/potential/PotentialData.h>
#include <spl/potential/IGeomOptimiser.h>
#include <spl/potential/IParameterisable.h>
#include <spl/potential/IPotential.h>

#include "common/PipeFunctions.h"
#include "common/StructureData.h"
#include "common/SharedData.h"
#include "common/UtilityFunctions.h"

// NAMESPACES ////////////////////////////////


namespace spipe {
namespace blocks {

namespace fs = ::boost::filesystem;
namespace common = ::spipe::common;
namespace utility = ::spipe::utility;

ParamPotentialGo::ParamPotentialGo(
	::spl::potential::IGeomOptimiserPtr optimiser,
  const bool writeOutput):
SpBlock("Parameterised potential geometry optimisation"),
PotentialGo(optimiser, writeOutput)
{
  init();
}

ParamPotentialGo::ParamPotentialGo(
	::spl::potential::IGeomOptimiserPtr optimiser,
  const ::spl::potential::OptimisationSettings & optimisationParams,
  const bool writeOutput):
SpBlock("Parameterised potential geometry optimisation"),
PotentialGo(optimiser, optimisationParams, writeOutput)
{
  init();
}

void ParamPotentialGo::pipelineStarting()
{
  // The pipeline is starting so try and get the potential parameters
  common::ObjectData<PotentialParams> params
    = common::getObject(common::GlobalKeys::POTENTIAL_PARAMS, getRunner()->memory());

  if(params.first != common::DataLocation::NONE)
  {
    setPotentialParams(*params.second);

    // The potential may have changed the params so reset them in the shared data
    common::setObject(
      common::GlobalKeys::POTENTIAL_PARAMS,
      params.first,
      myCurrentParams,
      getRunner()->memory()
    );

    if(!myCurrentParams.empty())
    {
      ::std::stringstream ss;
      ss << "params: " << myCurrentParams[0];
      for(size_t i = 1; i < myCurrentParams.size(); ++i)
      {
        ss << " " << myCurrentParams[i];
      }
      // Add a note to the table with the current parameter string
      getTableSupport().getTable().addTableNote(ss.str());
    }
  }

  // Call the parent to let them deal with the starting event too
  PotentialGo::pipelineStarting();
}

void ParamPotentialGo::in(spipe::common::StructureData & data)
{
  // Add the potential parameters to the structure data
  data.objectsStore[common::GlobalKeys::POTENTIAL_PARAMS] = myCurrentParams;

  // Let our parent class deal with it
  PotentialGo::in(data);
}

void ParamPotentialGo::init()
{
  SSLIB_ASSERT_MSG(
    getOptimiser().getPotential() != NULL && getOptimiser().getPotential()->getParameterisable() != NULL,
    "Must use geometry optimiser with parameterisable potential."
  );
  myParamPotential = getOptimiser().getPotential()->getParameterisable();
}

void ParamPotentialGo::setPotentialParams(const PotentialParams & params)
{
  // Need to get the actual parameters as the potential may use a combining rule or change
  // them in some way from those specified
	myParamPotential->setParams(params);
  myCurrentParams = myParamPotential->getParams();
}


}
}

