/*
 * Process.h
 *
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef PROCESS_H
#define PROCESS_H

// INCLUDES /////////////////////////////////////////////
#include "SSLib.h"

#include <string>
#include <vector>

// Tell boost not to try to link to the date_time library as interprocess
// only uses a portion that doesn't need compilation.
// See: http://www.boost.org/doc/libs/1_49_0/doc/html/interprocess.html#interprocess.intro.introduction_building_interprocess
#define BOOST_DATE_TIME_NO_LIB

#include <boost/filesystem/path.hpp>
#include <boost/interprocess/detail/os_thread_functions.hpp>
#include <boost/version.hpp>

// FORWARD DECLARES ////////////////////////////////

// DEFINES ////////////////////////////////////////

#if (BOOST_VERSION / 100000) <= 1 && ((BOOST_VERSION / 100) % 1000) <= 47
	#define	NS_BOOST_IPC_DETAIL boost::interprocess::detail
#else
	#define NS_BOOST_IPC_DETAIL boost::interprocess::ipcdetail
#endif

// FUNCTIONS ////////////////////////////////////////

namespace sstbx {
namespace os {

typedef NS_BOOST_IPC_DETAIL::OS_process_id_t ProcessId;

class Process
{
public:
  typedef ::std::vector< ::std::string> Arguments;
  struct Status
  {
    enum Value
    {
      READY,
      RUNNING,
      FINISHED
    };
  };

  struct RunResult
  {
    enum Value
    {
      SUCCESS,
      ERROR_EXE_NOT_FOUND,
      ERROR_RUN_FAILED
    };
  };

  Process(const ::std::string & exe);
  Process(const ::boost::filesystem::path & exe);

  RunResult::Value run();
  RunResult::Value run(const Arguments & argv);
  RunResult::Value runBlocking();
  RunResult::Value runBlocking(const Arguments & argv);
  bool waitTillFinished();

  bool stop();

  Status::Value getStatus() const;
  int getExitStatus();

private:

  bool run(const Arguments & argv, const bool blocking);
  void updateExitStatus();

  ::boost::filesystem::path myExe;
  Status::Value myStatus;
  int myExitStatus;
#ifdef SSLIB_OS_POSIX
  pid_t myProcessPid;
#endif
};

ProcessId getProcessId();

void parseParameters(
  ::std::vector< ::std::string> & outParams,
  const ::std::string & exeString
);

int runBlocking(const ::std::string & exe, const ::std::vector< ::std::string> & argv);
int runBlocking(const ::std::vector< ::std::string> & exeAndArgv);
int runBlocking(const ::boost::filesystem::path & exe, const ::std::vector< ::std::string> & argv);


}
}

#endif /* PROCESS_H */
