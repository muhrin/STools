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

#include <cstring>
#include <vector>

#include <armadillo>

#include <pipelib/AbstractSimpleStartBlock.h>
#include <pipelib/IDataSink.h>

// From SSTbx
#include <utility/MultiIdx.h>



// FORWARD DECLARATIONS ////////////////////////////////////
template <class DataType, class GlobalDataType>
class IPipeline;


namespace spipe { namespace blocks {

class PotentialParamSweep :
	public pipelib::AbstractSimpleStartBlock<StructureDataTyp, SharedDataTyp>,
	public pipelib::IDataSink<StructureDataTyp>
{
public:
	PotentialParamSweep(
		const ::arma::vec	&		from,
		const ::arma::vec	&		step,
		const ::arma::Col<unsigned int> & nSteps,
		SpPipelineTyp &				sweepPipeline);

	// From Block /////////////////////////////////
	virtual void pipelineInitialising();
	virtual void pipelineInitialised();
	virtual void start();
	// End from Block //////////////////////////////

  // From IDataSink /////////////////////////////
  virtual void in(StructureDataTyp * const data);
  // End from IDataSink /////////////////////////

private:

  void saveTableData(const StructureDataTyp & strData);

	size_t								              myNumParams;
	const ::arma::vec					          myFrom;
	const ::arma::vec					          myStep;
	const ::arma::Col<unsigned int>			myNSteps;
	::sstbx::utility::MultiIdx<size_t>	myStepExtents;

	SpPipelineTyp &                      mySweepPipeline;

	/** Buffer to store structure that have finished their path through the sub pipeline. */
	::std::vector<StructureDataTyp *>		myBuffer;

};

}}

#endif /* POTENTIAL_PARAM_SWEEP_H */
