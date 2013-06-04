/*
 * UtilFunctions.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: muhrin
 */

// INCLUDES //////////////////////////////////
#include "utility/UtilFunctions.h"

#include <sstream>

#include "math/Random.h"
#include "os/Process.h"

// NAMESPACES ////////////////////////////////

namespace sstbx {
namespace utility {

static const ::std::string charset = "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ1234567890";

char randomChar(const int seed = -1)
{
  if(seed == -1)
    return charset[math::randu(static_cast<int>(charset.length()))];
  else
    return charset[seed % charset.length()];
}

::std::string randomString(const size_t length)
{
  ::std::string result;
  result.resize(length);

  for(size_t i = 0; i < length; i++)
    result[i] = charset[math::randu(static_cast<int>(charset.length()))];

  return result;
}

std::string generateUniqueName(const ::std::string & prefix, const size_t randPostfixLength)
{
  // Build up the name
  std::stringstream ss;	//create a stringstream
  if(!prefix.empty())
    ss << prefix << "-";
  ss << os::getProcessId() << "-" << randomChar(static_cast<int>(time(NULL)));
  if(randPostfixLength > 0)
    ss << randomString(randPostfixLength);

  return ss.str();
}

std::string generateUniqueName(const size_t randPostfixLength)
{
  return generateUniqueName("", randPostfixLength);
}


}
}
