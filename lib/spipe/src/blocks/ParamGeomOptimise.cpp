/*
 * ParamGeomOptimise.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "blocks/ParamGeomOptimise.h"

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

namespace fs = boost::filesystem;
namespace common = spipe::common;
namespace utility = spipe::utility;

ParamGeomOptimise::ParamGeomOptimise(
    spl::potential::IGeomOptimiserPtr optimiser, const bool writeSummary) :
    Block("Parameterised geometry optimise"), GeomOptimise(optimiser,
        writeSummary)
{
  init();
}

ParamGeomOptimise::ParamGeomOptimise(
    spl::potential::IGeomOptimiserPtr optimiser,
    const spl::potential::OptimisationSettings & optimisationParams,
    const bool writeSummary) :
    Block("Parameterised potential geometry optimisation"), GeomOptimise(
        optimiser, optimisationParams, writeSummary)
{
  init();
}

void
ParamGeomOptimise::pipelineInitialising()
{
  myParamPotential->updateSpeciesDb(
      &getEngine()->globalData().getSpeciesDatabase());
}

void
ParamGeomOptimise::pipelineStarting()
{
  // Call the parent to let them deal with the starting event too
  GeomOptimise::pipelineStarting();

  // The pipeline is starting so try and get the potential parameters
  PotentialParams * const params = getEngine()->globalData().objectsStore.find(
      common::GlobalKeys::POTENTIAL_PARAMS);

  if(params)
  {
    setPotentialParams(*params);
    if(!myCurrentParams.empty())
    {
      // The potential may have changed the params so reset them in the shared data
      *params = myCurrentParams;

      std::stringstream ss;
      ss << "params: " << myCurrentParams[0];
      for(size_t i = 1; i < myCurrentParams.size(); ++i)
      {
        ss << " " << myCurrentParams[i];
      }
      // Add a note to the table with the current parameter string
      if(myWriteSummary)
        getTableSupport().getTable().addTableNote(ss.str());
    }
  }
}

void
ParamGeomOptimise::in(spipe::common::StructureData * const data)
{
  // Add the potential parameters to the structure data
  data->objectsStore[common::GlobalKeys::POTENTIAL_PARAMS] = myCurrentParams;

  // Let our parent class deal with it
  GeomOptimise::in(data);
}

void
ParamGeomOptimise::init()
{
  SSLIB_ASSERT_MSG(
      getOptimiser().getPotential() != NULL && getOptimiser().getPotential()->getParameterisable() != NULL,
      "Must use geometry optimiser with parameterisable potential.");
  myParamPotential = getOptimiser().getPotential()->getParameterisable();
}

void
ParamGeomOptimise::setPotentialParams(const PotentialParams & params)
{
  // Need to get the actual parameters as the potential may use a combining rule or change
  // them in some way from those specified
  myParamPotential->setParams(params);
  if(getEngine())
    myParamPotential->updateSpeciesDb(
        &getEngine()->globalData().getSpeciesDatabase());
  myCurrentParams = myParamPotential->getParams();
}

}
}

