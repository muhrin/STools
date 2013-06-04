/*
 * Process.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: muhrin
 */

// INCLUDES //////////////////////////////////
#include "os/Process.h"

#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <boost/smart_ptr/scoped_array.hpp>
#include <boost/tokenizer.hpp>

#ifdef SSLIB_OS_POSIX
extern "C"
{
#  include <sys/wait.h>
#  include <sys/types.h>
#  include <unistd.h>
}
#endif

// NAMESPACES ////////////////////////////////

namespace sstbx {
namespace os {

namespace fs = ::boost::filesystem;

Process::Process(const ::std::string & exe):
myExe(exe),
myStatus(Status::READY),
myExitStatus(-1)
#ifdef SSLIB_OS_POSIX
, myProcessPid(-1)
#endif
{}

Process::Process(const fs::path & exe):
myExe(exe),
myStatus(Status::READY),
myExitStatus(-1)
#ifdef SSLIB_OS_POSIX
, myProcessPid(-1)
#endif
{}

Process::RunResult::Value Process::run()
{
  if(!fs::exists(myExe))
    return RunResult::ERROR_EXE_NOT_FOUND;

  return RunResult::ERROR_RUN_FAILED;
}

Process::RunResult::Value Process::run(const Arguments & argv)
{
  if(!fs::exists(myExe))
    return RunResult::ERROR_EXE_NOT_FOUND;

  return RunResult::ERROR_RUN_FAILED;
}

Process::RunResult::Value Process::runBlocking()
{
  if(!fs::exists(myExe))
    return RunResult::ERROR_EXE_NOT_FOUND;

  if(os::runBlocking(myExe, Arguments()) == 0)
    return RunResult::SUCCESS;
  else
    return RunResult::ERROR_RUN_FAILED;
}

Process::RunResult::Value Process::runBlocking(const Arguments & argv)
{
  if(!fs::exists(myExe))
    return RunResult::ERROR_EXE_NOT_FOUND;

  if(os::runBlocking(myExe, argv) == 0)
    return RunResult::SUCCESS;
  else
    return RunResult::ERROR_RUN_FAILED;
}

bool Process::waitTillFinished()
{
  if(getStatus() == Status::READY)
    return false;

#ifdef SSLIB_OS_POSIX
  int status;
  waitpid(myProcessPid, &status, 0);
  if(WIFEXITED(status))
    myExitStatus = WEXITSTATUS(status);
#endif
  return true;
}

bool Process::stop()
{
  if(getStatus() != Status::RUNNING)
    return false;

#ifdef SSLIB_OS_POSIX
  kill(myProcessPid, SIGUSR1);
#else
  return false;
#endif
}

Process::Status::Value Process::getStatus() const
{
  return myStatus;
}

int Process::getExitStatus()
{
  updateExitStatus();
  return myExitStatus;
}

bool Process::run(const Arguments & argv, const bool blocking)
{
  if(!fs::exists(myExe))
    return false;

  ::boost::scoped_array<const char *> argvArray(new const char *[argv.size()  + 2]);
  argvArray[0] = myExe.string().c_str();
  for(size_t i = 0; i < argv.size(); ++i)
    argvArray[i + 1] = argv[i].c_str();

  argvArray[argv.size() + 1] = 0;
#ifdef SSLIB_OS_POSIX
  myProcessPid = fork();
  if(myProcessPid < 0)
  { // Failed to fork
    return false;
  }
  if(myProcessPid == 0)
  { // We are the child
    execvp(myExe.c_str(), const_cast<char **>(argvArray.get()));
    return true; // This line doesn't get executed
  }
  else
  { // We are the parent
    myStatus = Status::RUNNING;
    if(blocking)
    {
      waitpid(myProcessPid, 0, 0);
      myStatus = Status::FINISHED;
    }

    updateExitStatus();
    return true;
  }
#else
  return false;
#endif
}

void Process::updateExitStatus()
{
  if(getStatus() == Status::FINISHED)
  {
#ifdef SSLIB_OS_POSIX
    int status;
    waitpid(myProcessPid, &status, WNOHANG);
    if(WIFEXITED(status))
      myExitStatus = WEXITSTATUS(status);
#endif
  }
}

ProcessId getProcessId()
{
	return NS_BOOST_IPC_DETAIL::get_current_process_id();
}

void parseParameters(
  ::std::vector< ::std::string> & outParams,
  const ::std::string & exeString
)
{
  typedef boost::tokenizer<boost::char_separator<char> > Tok;
  const boost::char_separator<char> sep(" \t");

  Tok tok(exeString, sep);
  outParams.insert(outParams.begin(), tok.begin(), tok.end());
}

int runBlocking(const ::std::vector< ::std::string> & exeAndArgv)
{
  if(exeAndArgv.empty())
    return 1;

  const ::std::string exe = exeAndArgv[0];
  ::std::vector< ::std::string> args;
  for(size_t i = 1; i < exeAndArgv.size(); ++i)
  {
    args.push_back(exeAndArgv[i]);
  }
  return runBlocking(exe, args);
}

int runBlocking(const ::std::string & exe, const ::std::vector< ::std::string> & argv)
{
  ::boost::scoped_array<const char *> argvArray(new const char *[argv.size()  + 2]);
  argvArray[0] = exe.c_str();
  for(size_t i = 0; i < argv.size(); ++i)
    argvArray[i + 1] = argv[i].c_str();

  argvArray[argv.size() + 1] = 0;
#ifdef SSLIB_OS_POSIX
  const pid_t child = fork();
  if(child < 0)
  { // Failed to fork
    return 1;
  }
  if(child == 0)
  { // We are the child
    execvp(exe.c_str(), const_cast<char **>(argvArray.get()));
    return 0;
  }
  else
  { // We are the parent
    int childExitStatus;
    waitpid(child, &childExitStatus, 0);

    // Did the child exit normally?
    if(WIFEXITED(childExitStatus))
      return 0;
    else
      return 1;
  }
#else
  return 1;
#endif
}

int runBlocking(const ::boost::filesystem::path & exe, const ::std::vector< ::std::string> & argv)
{
  return runBlocking(exe.string(), argv);
}

}
}
