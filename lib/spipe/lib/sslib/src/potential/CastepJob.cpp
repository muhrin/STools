/*
 * CastepJob.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "potential/CastepJob.h"

#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/regex.hpp>
#include <boost/thread/thread.hpp>

#include "math/RunningStats.h"
#include "potential/CastepRun.h"

// NAMESPACES ////////////////////////////////
namespace sstbx {
namespace potential {

namespace fs = ::boost::filesystem;

os::Process::RunResult::Value CastepJob::runBlocking()
{
  return doRunBlocking();
}

os::Process::RunResult::Value CastepJob::run()
{
  return doRun();
}

bool CastepJob::stop()
{
  return getProcess().stop();
}

os::Process::RunResult::Value CastepJob::doRunBlocking()
{
  return getProcess().runBlocking(myArgs);
}

os::Process::RunResult::Value CastepJob::doRun()
{
  return getProcess().run(myArgs);
}

const CastepRun & CastepJob::getCastepRun() const
{
  return myCastepRun;
}

os::Process & CastepJob::getProcess()
{
  return *myProcess;
}

const os::Process & CastepJob::getProcess() const
{
  return *myProcess;
}

CastepJob::CastepJob(const ::std::string & runCommand, const CastepRun & castepRun):
myCastepRun(castepRun)
{
  os::parseParameters(myArgs, runCommand);
  fs::path exe;
  if(!myArgs.empty())
  {
    exe = myArgs[0];
    myArgs.erase(myArgs.begin());
  }
  myProcess.reset(new os::Process(exe));
  myArgs.push_back(myCastepRun.getSeed());
}

os::Process::RunResult::Value CastepGeomJob::run()
{
  using namespace ::boost::posix_time;

  if(myOptimisationListener)
  {
    const fs::path castepFile(getCastepRun().getCastepFile());
    fs::ifstream castepStream;
    // Try opening the file, maybe there is one from before
    if(fs::exists(castepFile))
    {
      castepStream.open(castepFile);
      castepStream.seekg(0, castepStream.end); // Move to the end
    }

    math::RunningStats stepTimingStats;
    time_duration sleepInterval = seconds(1);
    ptime stepStart(microsec_clock::universal_time()), stepEnd;

    const os::Process::RunResult::Value result = doRun();
    ::boost::posix_time::time_duration stepDuration;

    if(result != os::Process::RunResult::SUCCESS)
      return result;

    // If the file didn't exist before then wait for it to be created and open it
    if(!castepStream.is_open() && waitForCastepFile(castepFile))
      castepStream.open(castepFile);
    
    if(castepStream.is_open())
    {
      ::boost::optional<int> step;
      while(getProcess().getStatus() == os::Process::Status::RUNNING)
      {
        step = seekNextStep(castepStream);
        if(step)
        {
          stepEnd = microsec_clock::universal_time();
          stepTimingStats.insert(static_cast<double>((stepEnd - stepStart).seconds()));
          sleepInterval = seconds(static_cast<long>(stepTimingStats.mean() / 5.0));

          myOptimisationListener->finishedStep(*step - 1);
        }
        ::boost::this_thread::sleep(sleepInterval);
      }
      castepStream.close();
    }

    return os::Process::RunResult::SUCCESS;
  }
  else
    return runBlocking();
}

void CastepGeomJob::setOptimisationListener(ICastepGeomOptimisationListener * listener)
{
  myOptimisationListener = listener;
}

const ICastepGeomOptimisationListener * CastepGeomJob::getListener() const
{
  return myOptimisationListener;
}

CastepGeomJob::CastepGeomJob(const ::std::string & runCommand, const CastepRun & castepRun):
CastepJob(runCommand, castepRun)
{}

bool CastepGeomJob::waitForCastepFile(const ::boost::filesystem::path & castepFile) const
{
  bool found = false;
  while(getProcess().getStatus() == os::Process::Status::RUNNING)
  {
    if(fs::exists(castepFile))
    {
      found = true;
      break;
    }
    ::boost::this_thread::sleep(::boost::posix_time::seconds(1));
  }
  return found;
}

::boost::optional<int> CastepGeomJob::seekNextStep(::boost::filesystem::ifstream & castepStream) const
{
  static const ::boost::regex RE_ITERATION("Starting [[:word:]]+ iteration[[:blank:]]+([[:digit:]]+)");

  ::boost::optional<int> step;
  ::std::string line;
  ::boost::smatch match;
  while(::std::getline(castepStream, line) && ::boost::regex_search(line, match, RE_ITERATION))
  {
    if(match.size() > 1)
    {
      const ::std::string stepString(match[1].first, match[1].second);
      try
      {
        step.reset(::boost::lexical_cast<int>(stepString));
        break;
      }
      catch(const ::boost::bad_lexical_cast & /*e*/)
      {}
    }
  }
  return step;
}


}
}

