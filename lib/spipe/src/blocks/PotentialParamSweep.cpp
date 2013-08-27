/*
 * PotentialParamSweep.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "blocks/PotentialParamSweep.h"

#include <boost/lexical_cast.hpp>
#include <boost/foreach.hpp>


#include <spl/common/Structure.h>
#include <spl/utility/MultiIdxRange.h>
#include <spl/utility/UtilFunctions.h>

#include "common/PipeFunctions.h"
#include "common/SharedData.h"
#include "common/StructureData.h"
#include "common/UtilityFunctions.h"

// NAMESPACES ////////////////////////////////

namespace spipe {
namespace blocks {

namespace fs = ::boost::filesystem;
namespace ssc = ::spl::common;
namespace ssio = ::spl::io;
namespace ssu = ::spl::utility;
namespace structure_properties = ssc::structure_properties;
typedef common::GlobalKeys Keys;

const ::std::string PotentialParamSweep::POTPARAMS_FILE_EXTENSION("potparams");

PotentialParamSweep::PotentialParamSweep(
  const common::ParamRange & paramRange,
	SubpipePtr sweepPipeline):
SpBlock("Potential param sweep"),
myParamRange(paramRange),
mySweepPipeline(sweepPipeline),
myStepExtents(paramRange.nSteps.size())
{
	SP_ASSERT(
    (myParamRange.from.size() == myParamRange.step.size()) &&
    (myParamRange.from.size() == myParamRange.nSteps.size())
  );

	myNumParams = myParamRange.nSteps.size();
	for(size_t i = 0; i < myNumParams; ++i)
	{
		myStepExtents[i] = myParamRange.nSteps[i] + 1;
	}
}

void PotentialParamSweep::pipelineInitialising()
{
	// Set the parameters in the shared data
  getRunner()->memory().shared().objectsStore[Keys::POTENTIAL_SWEEP_RANGE] = myParamRange;

  myTableSupport.setFilename(
    common::getOutputFileStem(getRunner()->memory())
    + "." + POTPARAMS_FILE_EXTENSION
  );
  myTableSupport.registerRunner(*getRunner());
}

void PotentialParamSweep::start()
{
  ::std::string sweepPipeOutputPath;

  const ssu::MultiIdxRange<int> stepsRange(
    ParamSpaceIdx(myStepExtents.dims()),
    myStepExtents
  );

  PotentialParams params(myNumParams);
  BOOST_FOREACH(const ParamSpaceIdx & stepsIdx, stepsRange)
	{
    ::spipe::SharedDataType & sweepPipeSharedData = mySubpipeRunner->memory().shared();

		// Load the current potential parameters into the pipeline data
		for(size_t i = 0; i < myNumParams; ++i)
			params[i] = myParamRange.from[i] +
      static_cast<double>(stepsIdx[i]) * myParamRange.step[i];

		// Store the potential parameters in global memory
    getRunner()->memory().global().objectsStore[common::GlobalKeys::POTENTIAL_PARAMS] = params;

    // Set a directory for this set of parameters
    sweepPipeSharedData.appendToOutputDirName(ssu::generateUniqueName());

    // Get the relative path to where the pipeline write the structures to
    sweepPipeOutputPath = sweepPipeSharedData.getOutputPath(*mySubpipeRunner).string();

		// Run the sweep pipeline
		mySubpipeRunner->run();

		// Send any finished structure data down my pipe
		releaseBufferedStructures(sweepPipeOutputPath);
	}
}

void PotentialParamSweep::runnerAttached(SpRunnerSetup & setup)
{
  mySubpipeRunner = setup.createChildRunner(*mySweepPipeline);
	// Set outselves to collect any finished data from the sweep pipeline
	mySubpipeRunner->setFinishedDataSink(this);
}

void PotentialParamSweep::finished(SpStructureDataPtr data)
{
	// Copy over the parameters into the structure data
  const ::spipe::common::ObjectData<const PotentialParams> result = ::spipe::common::getObjectConst(
    ::spipe::common::GlobalKeys::POTENTIAL_PARAMS,
    mySubpipeRunner->memory()
  );

  if(result.first != common::DataLocation::NONE)
    data->objectsStore[common::GlobalKeys::POTENTIAL_PARAMS] = *result.second;

	// Register the data with our pipeline to transfer ownership
	// Save it in the buffer for sending down the pipe
	myBuffer.push_back(&getRunner()->registerData(data));
}

void PotentialParamSweep::releaseBufferedStructures(
  const ::spipe::utility::DataTable::Key & key
)
{
	// Send any finished structure data down my pipe
	BOOST_FOREACH(StructureDataType * const sweepStrData, myBuffer)
	{
    updateTable(key, *sweepStrData);

		out(*sweepStrData);
	}
	myBuffer.clear();
}

void PotentialParamSweep::updateTable(
  const utility::DataTable::Key & key,
  const StructureDataType & sweepStrData)
{
  utility::DataTable & table = myTableSupport.getTable();

  const PotentialParams * const params =
    sweepStrData.objectsStore.find(common::GlobalKeys::POTENTIAL_PARAMS);
  if(params)
  {
    // Update the table with the current parameters
    for(size_t i = 0; i < params->size(); ++i)
    {
      table.insert(
        key,
        "param" + ::boost::lexical_cast< ::std::string>(i),
        common::getString((*params)[i])
      );
    }
  }

  const ssc::Structure * const structure = sweepStrData.getStructure();
  if(structure)
  {
    const double * const internalEnergy = structure->getProperty(
      structure_properties::general::ENERGY_INTERNAL
      );
    if(internalEnergy)
    {
      const double energy = *internalEnergy;
      table.insert(
        key,
        "energy",
        common::getString(energy)
      );

      const size_t numAtoms = structure->getNumAtoms();
      table.insert(
        key,
        "energy/atom",
        common::getString(energy / numAtoms)
      );
    }

    const ssio::ResourceLocator locator = sweepStrData.getRelativeSavePath(*getRunner());
    if(!locator.empty())
    {
      table.insert(
        key,
        "lowest_path",
        locator.string()
      );
    }

    const unsigned int * const spacegroup = structure->getProperty(structure_properties::general::SPACEGROUP_NUMBER);
    if(spacegroup)
      table.insert(key, "sg", ::boost::lexical_cast< ::std::string>(*spacegroup));
  }
}

}
}

