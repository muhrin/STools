/*
 * Process.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: muhrin
 */

// INCLUDES //////////////////////////////////
#include "os/Process.h"

#include <iostream>

#include <boost/smart_ptr/scoped_array.hpp>

#if defined (__unix__) || (defined (__APPLE__) && defined (__MACH__))
#  define SSLIB_OS_POSIX
#endif

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


ProcessId getProcessId()
{
	return NS_BOOST_IPC_DETAIL::get_current_process_id();
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
