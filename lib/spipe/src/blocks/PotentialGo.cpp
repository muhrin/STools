/*
 * PotentialGo.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "PotentialGo.h"

#include <locale>

#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/tokenizer.hpp>

#include <pipelib/IPipeline.h>

// From SSTbx
#include <common/Structure.h>
#include <potential/StandardData.h>
#include <potential/IGeomOptimiser.h>

#include "common/StructureData.h"
#include "common/UtilityFunctions.h"

// NAMESPACES ////////////////////////////////


namespace spipe
{
namespace blocks
{

PotentialGo::PotentialGo(
	const sstbx::potential::IGeomOptimiser & optimiser,
  const ::arma::mat33 * const              externalPressure):
pipelib::Block<StructureDataTyp, SharedDataTyp>("Potential geometry optimisation"),
myOptimiser(optimiser)
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

void PotentialGo::in(spipe::common::StructureData & data)
{
  ::boost::shared_ptr<sstbx::potential::StandardData<> > optData;
	
  if(myOptimiser.optimise(*data.getStructure(), optData, &myExternalPressure))
  {
	  // Copy over information from the optimisation results
	  data.enthalpy.reset(optData->totalEnthalpy);

	  myOutput->in(data);
  }
  else
  {
    // The structure failed to geometry optimise properly so drop it
    myPipeline->dropData(data);
  }
}

}}

