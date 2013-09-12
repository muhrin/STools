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
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <boost/interprocess/sync/sharable_lock.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/thread/locks.hpp>
#include <boost/tokenizer.hpp>

#include <spl/io/BoostFilesystem.h>
#include <spl/utility/UtilFunctions.h>

// Local includes
#include "common/PipeFunctions.h"
#include "utility/DataTableInserters.h"

// NAMESPACES ////////////////////////////////

namespace spipe {
namespace blocks {

namespace fs = ::boost::filesystem;
namespace ip = ::boost::interprocess;

namespace splu = ::spl::utility;

const ::std::string RunPotentialParamsQueue::DEFAULT_PARAMS_QUEUE_FILE =
    "params_queue";
const ::std::string RunPotentialParamsQueue::DEFAULT_PARAMS_DONE_FILE =
    "params_done";
const ::std::string RunPotentialParamsQueue::POTPARAMS_FILE_EXTENSION =
    "potparams";

RunPotentialParamsQueue::RunPotentialParamsQueue(BlockHandle & sweepPipeline) :
    Block("Run potential params queue"), mySweepPipeline(sweepPipeline), mySubpipeEngine(
        NULL), myQueueFile(DEFAULT_PARAMS_QUEUE_FILE), myDoneFile(
        DEFAULT_PARAMS_DONE_FILE)
{
  if(!fs::exists(myQueueFile))
    spl::io::createFile(myQueueFile);
  myQueueFileLock = ip::file_lock(myQueueFile.c_str());
  if(!exists(myDoneFile))
    spl::io::createFile(myDoneFile);
  myDoneFileLock = ip::file_lock(myDoneFile.c_str());
}

RunPotentialParamsQueue::RunPotentialParamsQueue(
    const ::std::string * const queueFile, const ::std::string * const doneFile,
    BlockHandle & sweepPipeline) :
    Block("Run potential params queue"), mySweepPipeline(sweepPipeline), mySubpipeEngine(
        NULL), myQueueFile(queueFile ? *queueFile : DEFAULT_PARAMS_QUEUE_FILE), myDoneFile(
        doneFile ? *doneFile : DEFAULT_PARAMS_DONE_FILE)
{
  if(!fs::exists(myQueueFile))
    spl::io::createFile(myQueueFile);
  myQueueFileLock = ip::file_lock(myQueueFile.c_str());
  if(!exists(myDoneFile))
    spl::io::createFile(myDoneFile);
  myDoneFileLock = ip::file_lock(myDoneFile.c_str());
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
      myCurrentParams = myParamsQueue.front();

      // Store the potential parameters in shared memory
      mySubpipeEngine->sharedData().objectsStore[common::GlobalKeys::POTENTIAL_PARAMS] =
          myCurrentParams;

      // Set a directory for this set of parameters
      mySubpipeEngine->sharedData().appendToOutputDirName(
          splu::generateUniqueName());

      // Get the relative path to where the pipeline write the structures to
      // Need to store this now as the shared data is not guaranteed to be
      // available after the subpipe runs
      const ::std::string subpipeOutputPath =
          mySubpipeEngine->sharedData().getOutputPath().string();

      mySubpipeEngine->run();

      myDoneParams.push_back(myCurrentParams);
      myParamsQueue.pop();

      // Send the resultant structures down our pipe
      releaseBufferedStructures(subpipeOutputPath);
    }
    updateDoneParams();
  }
}

bool
RunPotentialParamsQueue::getWork()
{
  if(!fs::exists(myQueueFile))
    return false;

  ip::scoped_lock< ip::file_lock> lockQueue(myQueueFileLock);

  fs::fstream queueStream(myQueueFile);
  ::std::string line;
  bool foundParams = false;
  for(int i = 0; i < 10; ++i)
  {
    if(!::std::getline(queueStream, line))
      break;
    const ::boost::optional< Params> & params = readParams(line);
    if(params)
    {
      foundParams = true;
      myParamsQueue.push(*params);
    }
  }

  if(foundParams)
  {
    // Now we need to remove the lines we took as work from the file so save the rest
    // of the file to a buffer
    ::std::stringstream sout;
    ::std::copy(::std::istreambuf_iterator< char>(queueStream),
        ::std::istreambuf_iterator< char>(),
        ::std::ostreambuf_iterator< char>(sout));

    queueStream.close();
    queueStream.open(myQueueFile, ::std::fstream::out | ::std::fstream::trunc);

    ::std::copy(::std::istreambuf_iterator< char>(sout),
        ::std::istreambuf_iterator< char>(),
        ::std::ostreambuf_iterator< char>(queueStream));
  }

  queueStream.close();

  return foundParams;
}

void
RunPotentialParamsQueue::writeParams(const Params & params,
    ::std::ostream & os) const
{
  for(int i = 0; i < params.size(); ++i)
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
  // Update the file of finish parameters
  ip::scoped_lock< ip::file_lock> lockQueue(myDoneFileLock);

  fs::ofstream doneStream(myDoneFile, std::ios::out | std::ios::app);
  ::std::for_each(myDoneParams.begin(), myDoneParams.end(),
      ::boost::bind(&RunPotentialParamsQueue::writeParams, this, _1,
          ::boost::ref(doneStream)));

  doneStream.close();
  myDoneParams.clear();
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

