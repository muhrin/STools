/*
 * RunPotentialParamsQueue.h
 *
 *  Created on: Sep 11, 2013
 *      Author: Martin Uhrin
 */

#ifndef RUN_POTENTIAL_PARAMS_QUEUE_H
#define RUN_POTENTIAL_PARAMS_QUEUE_H

// INCLUDES /////////////////////////////////////////////
#include <ostream>
#include <queue>
#include <string>
#include <vector>

#include <boost/filesystem/path.hpp>
#include <boost/noncopyable.hpp>
#include <boost/optional.hpp>

// Local includes
#include "SpTypes.h"
#include "utility/DataTableSupport.h"

// FORWARD DECLARATIONS ////////////////////////////////////

namespace spl {
namespace common {
class Structure;
}
}

namespace spipe {
namespace blocks {

class RunPotentialParamsQueue : public StartBlock, public FinishedSink, ::boost::noncopyable
{
public:
  static const ::std::string DEFAULT_PARAMS_QUEUE_FILE;
  static const ::std::string DEFAULT_PARAMS_DONE_FILE;
  static const ::std::string POTPARAMS_FILE_EXTENSION;

  explicit RunPotentialParamsQueue(BlockHandle & sweepPipeline);
  RunPotentialParamsQueue(const ::std::string * const queueFile,
      const ::std::string * const doneFile, BlockHandle & sweepPipeline);

  // From Block ///////////////////////////////
  virtual void
  engineAttached(EngineSetup * setup);
  virtual void
  engineDetached();
  virtual void
  pipelineInitialising();
  // End From Block ///////////////////////////

  // From StartBlock ///
  virtual void
  start();
  // End from StartBlock
private:
  typedef ::std::vector<double> Params;

  // From FinishedSink ///////////////////////
  virtual void
  finished(StructureDataUniquePtr data);
  // End from FinishedSink //////////////////

  bool getWork();
  void writeParams(const Params & params, ::std::ostream & os) const;
  ::boost::optional< Params> readParams(const ::std::string & paramsLine) const;
  void
  updateDoneParams();

  void
  releaseBufferedStructures(
      const ::spipe::utility::DataTable::Key & key);
  void
  updateTable(const utility::DataTable::Key & key,
      const StructureDataType * const structureData);

  BlockHandle mySweepPipeline;
  Engine * mySubpipeEngine;

  const ::boost::filesystem::path myQueueFile;
  const ::boost::filesystem::path myDoneFile;

  Params myCurrentParams;

  ::std::queue< Params> myParamsQueue;
  ::std::vector< Params> myDoneParams;

  ::std::vector< StructureDataType *> myBuffer;

  ::spipe::utility::DataTableSupport myTableSupport;
};

}
}

#endif /* RUN_POTENTIAL_PARAMS_QUEUE_H */
