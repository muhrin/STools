/*
 * PotentialParamSweep.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "blocks/PotentialParamSweep.h"

#include <boost/foreach.hpp>

// From SSTbx
#include <utility/Loops.h>

// From PipelineLib
#include <pipelib/IPipeline.h>

#include "common/SharedData.h"
#include "common/StructureData.h"
#include "common/PipeFunctions.h"
#include "common/UtilityFunctions.h"

// NAMESPACES ////////////////////////////////

namespace spipe
{
namespace blocks
{

PotentialParamSweep::PotentialParamSweep(
	const ::arma::vec	&		from,
	const ::arma::vec	&		step,
	const ::arma::Col<unsigned int> & nSteps,
	SpPipelineTyp &				sweepPipeline):
pipelib::Block<StructureDataTyp, SharedDataTyp>("Potential param sweep"),
myFrom(from),
myStep(step),
myNSteps(nSteps),
mySweepPipeline(sweepPipeline),
myStepExtents(nSteps.n_rows)
{
	SP_ASSERT((myFrom.n_rows == myStep.n_rows) && (myFrom.n_rows == myNSteps.n_rows));

	myNumParams		= myNSteps.n_rows;
	for(size_t i = 0; i < myNumParams; ++i)
	{
		myStepExtents[i] = myNSteps(i);
	}
}

void PotentialParamSweep::pipelineInitialising()
{
	// Set the parameters in the shared data
	SharedDataTyp & sharedDat = myPipeline->getSharedData();
	sharedDat.potSweepFrom.reset(myFrom);
	sharedDat.potSweepStep.reset(myStep);
	sharedDat.potSweepNSteps.reset(myNSteps);
}

void PotentialParamSweep::pipelineInitialised()
{
	// Set outselves to collect any finished data from the sweep pipeline
	mySweepPipeline.setFinishedDataSink(*this);
}

void PotentialParamSweep::start()
{
  namespace common = ::spipe::common;
	using ::sstbx::utility::Loops;

	for(Loops<size_t> loops(myStepExtents); !loops.isAtEnd(); ++loops)
	{
		// Load the current potential parameters into the pipeline data
		::arma::vec params(myNumParams);
		for(size_t i = 0; i < myNumParams; ++i)
		{
			params(i) = myFrom(i) + (*loops)[i] * myStep(i);
		}
		// Store the potential parameters in global memory
    myPipeline->getGlobalData().objectsStore.insert(::spipe::common::GlobalKeys::POTENTIAL_PARAMS, params);

    // Set a directory for this set of parameters
    mySweepPipeline.getSharedData().appendToOutputDirName(common::generateUniqueName());

		// Start the sweep pipeline
		mySweepPipeline.start();


		// Send any finished structure data down my pipe
		BOOST_FOREACH(StructureDataTyp * const sweepPipeData, myBuffer)
		{
			myOutput->in(*sweepPipeData);
		}
		myBuffer.clear();
	}
}

void  PotentialParamSweep::in(StructureDataTyp * const data)
{
	SP_ASSERT(data);

	// Copy over the parameters into the structure data
  const ::spipe::common::ObjectData<const ::arma::vec> result = ::spipe::common::getObjectConst(
    ::spipe::common::GlobalKeys::POTENTIAL_PARAMS,
    mySweepPipeline
  );

  if(result.first != ::spipe::common::DataLocation::NONE)
  {
    data->objectsStore.insert(::spipe::common::GlobalKeys::POTENTIAL_PARAMS, *result.second);
  }

	// Register the data with our pipeline to transfer ownership
	myPipeline->registerNewData(data);

  saveTableData(*data);

	// Save it in the buffer for sending down the pipe
	myBuffer.push_back(data);
}

void PotentialParamSweep::saveTableData(const StructureDataTyp & strData)
{
  using ::spipe::common::GlobalKeys;

  ::spipe::common::DataTable & table = myPipeline->getSharedData().dataTable;

  const ::arma::vec * const params = strData.objectsStore.find(GlobalKeys::POTENTIAL_PARAMS);
  if(params)
  {
    ::std::stringstream paramStream;
    paramStream << *params;
    table.insert(*strData.name, "pot_params", paramStream.str());
  }
}

}}

