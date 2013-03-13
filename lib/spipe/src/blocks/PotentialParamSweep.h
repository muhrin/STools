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

#include <boost/noncopyable.hpp>
#include <boost/shared_ptr.hpp>

#include <armadillo>

#include <pipelib/pipelib.h>

// From SSTbx
#include <utility/MultiIdx.h>

// Local includes
#include "SpTypes.h"
#include "common/CommonData.h"
#include "utility/DataTable.h"
#include "utility/DataTableSupport.h"

namespace spipe {

// FORWARD DECLARATIONS ////////////////////////////////////
namespace common {
class DataTableWriter;
}

namespace blocks {

class PotentialParamSweep : public SpStartBlock, public SpFinishedSink, ::boost::noncopyable
{
public:
  typedef ::sstbx::UniquePtr< ::spipe::SpPipe>::Type SubpipePtr;

  static const ::std::string POTPARAMS_FILE_EXTENSION;

	PotentialParamSweep(const common::ParamRange & paramRange, SubpipePtr sweepPipeline);

	// From Block /////////////////////////////////
	virtual void start();
	// End from Block //////////////////////////////

private:

  // From Block ///////////////////////////////
  virtual void runnerAttached(SpRunnerSetup & setup);
  virtual void pipelineInitialising();
  // End From Block ///////////////////////////

  // From FinishedSink ///////////////////////
  virtual void finished(SpStructureDataPtr data);
  // End from FinishedSink //////////////////

  void releaseBufferedStructures(
    const ::spipe::utility::DataTable::Key & key
  );

  void updateTable(
    const utility::DataTable::Key & key,
    const StructureDataType & sweepStrData
  );

	size_t								                    myNumParams;
  const common::ParamRange                  myParamRange;
	::sstbx::utility::MultiIdx<unsigned int>	myStepExtents;

  ::spipe::utility::DataTableSupport  myTableSupport;

	SubpipePtr mySweepPipeline;

	/** Buffer to store structure that have finished their path through the sub pipeline. */
	::std::vector<StructureDataType *>		myBuffer;
  SpChildRunnerPtr                    mySubpipeRunner;

};

}}

#endif /* POTENTIAL_PARAM_SWEEP_H */
