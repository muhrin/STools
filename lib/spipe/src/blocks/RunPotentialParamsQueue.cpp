/*
 * RunPotentialParamsQueue.cpp
 *
 *  Created on: Sep 11, 2013
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "blocks/RunPotentialParamsQueue.h"

#include <algorithm>

#include <boost/bind.hpp>
#include <boost/foreach.hpp>
#include <boost/interprocess/sync/file_lock.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/thread/locks.hpp>
#include <boost/tokenizer.hpp>

#include <spl/io/BoostFilesystem.h>
#include <spl/os/Process.h>
#include <spl/utility/Armadillo.h>

// Local includes
#include "common/PipeFunctions.h"
#include "utility/DataTableInserters.h"

// NAMESPACES ////////////////////////////////

namespace spipe {
namespace blocks {

namespace fs = ::boost::filesystem;
namespace ip = ::boost::interprocess;
namespace posix_time = ::boost::posix_time;

namespace splu = ::spl::utility;

const ::std::string RunPotentialParamsQueue::DEFAULT_PARAMS_QUEUE_FILE =
    "params_queue";
const ::std::string RunPotentialParamsQueue::DEFAULT_PARAMS_DONE_FILE =
    "params_done";
const ::std::string RunPotentialParamsQueue::POTPARAMS_FILE_EXTENSION =
    "potparams";

// The number of work items to take the first time around, after that it will
// be determined according to a target time to finish each chunk
static const size_t INITIAL_NUM_WORK_ITEMS = 20;
static const posix_time::time_duration DEFAULT_TARGET_CHUNK_TIME =
    posix_time::minutes(10);

RunPotentialParamsQueue::RunPotentialParamsQueue(BlockHandle & sweepPipeline) :
    Block("Run potential params queue"), mySweepPipeline(sweepPipeline), mySubpipeEngine(
    NULL), myQueueFile(DEFAULT_PARAMS_QUEUE_FILE), myDoneFile(
        DEFAULT_PARAMS_DONE_FILE), myTargetChunkTime(DEFAULT_TARGET_CHUNK_TIME)
{
  myNumWorkItemsChunk = INITIAL_NUM_WORK_ITEMS;
}

RunPotentialParamsQueue::RunPotentialParamsQueue(
    const ::std::string * const queueFile, const ::std::string * const doneFile,
    BlockHandle & sweepPipeline) :
    Block("Run potential params queue"), mySweepPipeline(sweepPipeline), mySubpipeEngine(
    NULL), myQueueFile(queueFile ? *queueFile : DEFAULT_PARAMS_QUEUE_FILE), myDoneFile(
        doneFile ? *doneFile : DEFAULT_PARAMS_DONE_FILE), myTargetChunkTime(
        DEFAULT_TARGET_CHUNK_TIME)
{
  myNumWorkItemsChunk = INITIAL_NUM_WORK_ITEMS;
}

void
RunPotentialParamsQueue::engineAttached(EngineSetup * setup)
{
  mySubpipeEngine = setup->createEngine();
  mySubpipeEngine->setFinishedDataSink(this);
  mySubpipeEngine->attach(mySweepPipeline);
}

void
RunPotentialParamsQueue::engineDetached()
{
  if(mySubpipeEngine)
  {
    mySubpipeEngine->detach();
    mySubpipeEngine->setFinishedDataSink(NULL);
    mySubpipeEngine = NULL;
  }
}

void
RunPotentialParamsQueue::pipelineInitialising()
{
  myTableSupport.setFilename(
      common::getOutputFileStem(getEngine()->sharedData(),
          getEngine()->globalData()) + "." + POTPARAMS_FILE_EXTENSION);
  myTableSupport.registerEngine(getEngine());
}

void
RunPotentialParamsQueue::finished(StructureDataUniquePtr data)
{
  if(!data->objectsStore.find(common::GlobalKeys::POTENTIAL_PARAMS))
    data->objectsStore[common::GlobalKeys::POTENTIAL_PARAMS] = myCurrentParams;

  // Register the data with our pipeline to transfer ownership
  // Save it in the buffer for sending down the pipe
  myBuffer.push_back(getEngine()->registerData(data));
}

void
RunPotentialParamsQueue::start()
{
  while(getWork())
  {
    while(!myParamsQueue.empty())
    {
      const posix_time::ptime startTime =
          posix_time::microsec_clock::universal_time();

      myCurrentParams = myParamsQueue.front();

      // Store the potential parameters in shared memory
      mySubpipeEngine->sharedData().objectsStore[common::GlobalKeys::POTENTIAL_PARAMS] =
          myCurrentParams;

      // Set a directory for this set of parameters
      mySubpipeEngine->sharedData().appendToOutputDirName(
          generateParamDirName(myCurrentParams));

      // Get the relative path to where the pipeline write the structures to
      // Need to store this now as the shared data is not guaranteed to be
      // available after the subpipe runs
      const ::std::string subpipeOutputPath =
          mySubpipeEngine->sharedData().getOutputPath().string();

      mySubpipeEngine->run();

      myDoneParams.push_back(myCurrentParams);
      myParamsQueue.pop();

      // Save how long it took to process that parameter set
      myWorkItemsTiming.insert(
          posix_time::microsec_clock::universal_time() - startTime);

      // Send the resultant structures down our pipe
      releaseBufferedStructures(subpipeOutputPath);
    }
    updateDoneParams();
    updateWorkChunkSize();
  }
}

bool
RunPotentialParamsQueue::getWork()
{
  if(!fs::exists(myQueueFile))
    return false;

  ip::file_lock lock(myQueueFile.c_str());
  ip::scoped_lock< ip::file_lock> lockQueue(lock);

  fs::fstream queueStream(myQueueFile);
  ::std::stringstream takenWorkItems, originalContents;
  ::std::string line;
  size_t numParamsRead = 0;
  while(numParamsRead < myNumWorkItemsChunk)
  {
    if(!::std::getline(queueStream, line))
      break;

    bool takenWork = false;
    if(!line.empty() && line[0] != '#')
    {
      const ::boost::optional< Params> & params = readParams(line);
      if(params)
      {
        ++numParamsRead;
        myParamsQueue.push(*params);
        takenWorkItems << "#" << spl::os::getProcessId() << " " << line << "\n";
        takenWork = true;
      }
    }
    if(!takenWork)
      originalContents << line << "\n";
  }

  if(numParamsRead > 0)
  {
    // Save the rest of the file to buffer
    ::std::copy(::std::istreambuf_iterator< char>(queueStream),
        ::std::istreambuf_iterator< char>(),
        ::std::ostreambuf_iterator< char>(originalContents));

    // Go back to the start of the file
    queueStream.clear(); // Clear the EoF flag
    queueStream.seekg(0, ::std::ios::beg);

    // Write out the whole new contents
    // First the rest of the file
    ::std::copy(::std::istreambuf_iterator< char>(originalContents),
        ::std::istreambuf_iterator< char>(),
        ::std::ostreambuf_iterator< char>(queueStream));
    // Then the part of the queue that we're running
    ::std::copy(::std::istreambuf_iterator< char>(takenWorkItems),
        ::std::istreambuf_iterator< char>(),
        ::std::ostreambuf_iterator< char>(queueStream));
  }
  queueStream.close();

  return numParamsRead > 0;
}

void
RunPotentialParamsQueue::writeParams(const Params & params,
    ::std::ostream & os) const
{
  for(size_t i = 0; i < params.size(); ++i)
    os << params[i] << " ";
  os << "\n";
}

::boost::optional< RunPotentialParamsQueue::Params>
RunPotentialParamsQueue::readParams(const ::std::string & paramsLine) const
{
  typedef boost::tokenizer< boost::char_separator< char> > Toker;
  const boost::char_separator< char> tokSep(" \t");

  Params params;
  Toker toker(paramsLine, tokSep);
  BOOST_FOREACH(const ::std::string & tok, toker)
  {
    try
    {
      params.push_back(::boost::lexical_cast< double>(tok));
    }
    catch(const ::boost::bad_lexical_cast & /*e*/)
    {
    }
  }

  if(params.empty())
    return ::boost::optional< Params>();

  return params;
}

void
RunPotentialParamsQueue::updateDoneParams()
{
  if(!fs::exists(myDoneFile))
    return;

  // Update the file of finish parameters
  ip::file_lock lock(myDoneFile.c_str());
  ip::scoped_lock< ip::file_lock> lockQueue(lock);

  fs::ofstream doneStream(myDoneFile, std::ios::out | std::ios::app);
  ::std::for_each(myDoneParams.begin(), myDoneParams.end(),
      ::boost::bind(&RunPotentialParamsQueue::writeParams, this, _1,
          ::boost::ref(doneStream)));

  doneStream.close();
  myDoneParams.clear();
}

void
RunPotentialParamsQueue::updateWorkChunkSize()
{
  const posix_time::time_duration meanDuration = myWorkItemsTiming.mean();
  if(meanDuration > myTargetChunkTime)
  {
    // One work item takes longer than our target time so just do one
    // at a time
    myNumWorkItemsChunk = 1;
    return;
  }

  // Can't divide two times so keep multiplying up until we get the duration we want
  int i =
      meanDuration * myNumWorkItemsChunk > myTargetChunkTime ?
          1 : myNumWorkItemsChunk + 1;
  for(; meanDuration * i < myTargetChunkTime; ++i)
  { // Nothing to do
  }
  myNumWorkItemsChunk = i;
}

::std::string
RunPotentialParamsQueue::generateParamDirName(const ::arma::vec & params) const
{
  static const ::boost::hash< ::arma::vec> VEC_HASHER = ::boost::hash< ::arma::vec>();

  std::stringstream stream;
  stream << getEngine()->globalData().getSeedName() << "-" << std::hex
      << VEC_HASHER(params);
  return stream.str();
}

void
RunPotentialParamsQueue::releaseBufferedStructures(
    const ::spipe::utility::DataTable::Key & key)
{
  ::std::for_each(myBuffer.begin(), myBuffer.end(),
      ::boost::bind(&RunPotentialParamsQueue::updateTable, this,
          ::boost::ref(key), _1));

  // Send any finished structure data down my pipe
  BOOST_FOREACH(StructureDataType * const sweepStrData, myBuffer)
  {
    out(sweepStrData);
  }
  myBuffer.clear();
}

void
RunPotentialParamsQueue::updateTable(const utility::DataTable::Key & key,
    const StructureDataType * const structureData)
{
  utility::insertStructureInfoAndPotentialParams(key, *structureData,
      getEngine()->sharedData().getOutputPath(), myTableSupport.getTable());
}

}
}

