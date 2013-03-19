/*
 * UtilFunctions.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: muhrin
 */

// INCLUDES //////////////////////////////////
#include "utility/UtilFunctions.h"
#include "math/Random.h"

#include <sstream>

// NAMESPACES ////////////////////////////////

namespace sstbx {
namespace utility {

::std::string randomString(const size_t length)
{
  static const ::std::string charset =
    "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ1234567890";

  ::std::string result;
  result.resize(length);

  for(size_t i = 0; i < length; i++)
    result[i] = charset[math::rand(static_cast<int>(charset.length()))];

  return result;
}

ProcessId getProcessId()
{
	return NS_BOOST_IPC_DETAIL::get_current_process_id();
}

std::string generateUniqueName()
{
  // Use boost as portable way to get the process id
  const ProcessId processId = getProcessId();
  const time_t currTime = time(NULL);
 
  // Build up the name
  std::stringstream ss;	//create a stringstream
  ss << processId << "-" << currTime << "-" << randomString(4);

  return ss.str();
}


}
}
