/*
 * PotentialParamSweep.h
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef POTENTIAL_PARAM_SWEEP_H
#define POTENTIAL_PARAM_SWEEP_H

// INCLUDES /////////////////////////////////////////////
#include "StructurePipe.h"

#include <vector>

#include <boost/shared_ptr.hpp>

#include <armadillo>

#include <pipelib/pipelib.h>

// From SSTbx
#include <utility/MultiIdx.h>

// Local includes
#include "PipeLibTypes.h"
#include "utility/DataTable.h"
#include "utility/DataTableSupport.h"

namespace spipe {

// FORWARD DECLARATIONS ////////////////////////////////////
namespace common {
class DataTableWriter;
}

namespace blocks
{

class PotentialParamSweep :
	public pipelib::StartBlock<StructureDataTyp, SharedDataTyp, SharedDataTyp>,
	public pipelib::FinishedSink<StructureDataTyp>
{
public:
	PotentialParamSweep(
		const ::arma::vec	&		from,
		const ::arma::vec	&		step,
		const ::arma::Col<unsigned int> & nSteps,
		SpStartBlockTyp &     sweepPipeline);

	// From Block /////////////////////////////////
	virtual void pipelineInitialising();
	virtual void start();
	// End from Block //////////////////////////////

private:

  // From Block ///////////////////////////////
  virtual void runnerAttached(SpRunnerSetup & setup);
  // End From Block ///////////////////////////

  // From FinishedSink ///////////////////////
  virtual void finished(SpStructureDataPtr data);
  // End from FinishedSink //////////////////

  void releaseBufferedStructures(
    const ::spipe::utility::DataTable::Key & key
  );

  void updateTable(
    const utility::DataTable::Key & key,
    const StructureDataTyp & sweepStrData
  );

	size_t								                    myNumParams;
	const ::arma::vec					                myFrom;
	const ::arma::vec					                myStep;
	const ::arma::Col<unsigned int>			      myNSteps;
	::sstbx::utility::MultiIdx<unsigned int>	myStepExtents;

  ::spipe::utility::DataTableSupport  myTableSupport;

	SpStartBlockTyp &                   mySweepPipeline;

	/** Buffer to store structure that have finished their path through the sub pipeline. */
	::std::vector<StructureDataTyp *>		myBuffer;
  SpChildRunnerPtr                    mySubpipeRunner;

};

}}

#endif /* POTENTIAL_PARAM_SWEEP_H */
