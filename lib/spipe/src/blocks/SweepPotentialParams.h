/*
 * SweepPotentialParams.h
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef SWEEP_POTENTIAL_PARAMS_H
#define SWEEP_POTENTIAL_PARAMS_H

// INCLUDES /////////////////////////////////////////////
#include "StructurePipe.h"

#include <vector>

#include <boost/noncopyable.hpp>
#include <boost/shared_ptr.hpp>

#include <armadillo>

#include <pipelib/pipelib.h>

#include <spl/utility/MultiIdx.h>

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

class SweepPotentialParams : public StartBlock, public FinishedSink, ::boost::noncopyable
{
public:
  static const ::std::string POTPARAMS_FILE_EXTENSION;

  SweepPotentialParams(const common::ParamRange & paramRange, BlockHandle sweepPipeline);

  // From Block /////////////////////////////////
  virtual void
  start();
  // End from Block //////////////////////////////

private:
  typedef ::spl::utility::MultiIdx< int> ParamSpaceIdx;
  typedef ::std::vector< double> PotentialParams;

  // From Block ///////////////////////////////
  virtual void
  engineAttached(EngineSetup * setup);
  virtual void
  engineDetached();
  virtual void
  pipelineInitialising();
  // End From Block ///////////////////////////

  // From FinishedSink ///////////////////////
  virtual void
  finished(StructureDataUniquePtr data);
  // End from FinishedSink //////////////////

  void
  releaseBufferedStructures(const ::spipe::utility::DataTable::Key & key);

  void
  updateTable(const utility::DataTable::Key & key, const StructureDataType & sweepStrData);

  size_t myNumParams;
  const common::ParamRange myParamRange;
  ParamSpaceIdx myStepExtents;

  ::spipe::utility::DataTableSupport myTableSupport;

  BlockHandle mySweepPipeline;

  /** Buffer to store structure that have finished their path through the sub pipeline. */
  ::std::vector< StructureDataType *> myBuffer;
  Engine * mySubpipeEngine;

};

}
}

#endif /* POTENTIAL_PARAM_SWEEP_H */
