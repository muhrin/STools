/*
 * Process.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: muhrin
 */

// INCLUDES //////////////////////////////////
#include "os/Process.h"

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
  ::boost::scoped_array<const char *> argvArray(new const char *[argv.size()  + 1]);
  for(size_t i = 0; i < argv.size(); ++i)
  {
    argvArray[i] = argv[i].c_str();
  }
  argvArray[argv.size()] = 0;
#ifdef SSLIB_OS_POSIX
  const pid_t child = fork();
  if (child < 0)
  { // Failed to fork
    return 1;
  }
  if (child == 0)
  { // We are the parent
    wait(NULL);
    return 0;
  }
  else
  { // We are the child
    execv(exe.c_str(), const_cast<char **>(argvArray.get()));
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
