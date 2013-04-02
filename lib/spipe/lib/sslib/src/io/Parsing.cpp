/*
 * Parsing.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "io/Parsing.h"

#include <sstream>

#include <boost/algorithm/string/find.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/regex.hpp>

// DEFINES /////////////////////////////////


// NAMESPACES ////////////////////////////////


namespace sstbx {
namespace io {

bool findFirstLine(
  ::std::string & matchingLine,
  ::std::istream & inputStream,
  const ::std::string & token
)
{
  ::std::string line;
  while(::std::getline(inputStream, line))
  {
    if(::boost::find_first(line, token))
    {
      matchingLine = line;
      return true;
    }
  }
  return false;
}

bool findFirstFloat(
  double & number,
  const ::std::string & line
)
{
  static const ::boost::regex RE_FLOAT("([-+]?[0-9]*\\.?[0-9]+([eE][-+]?[0-9]+)?)");

  ::boost::smatch match;
  if(::boost::regex_search(line, match, RE_FLOAT))
  {
    const ::std::string numString(match[1].first, match[1].second);;
    try
    {
      number = ::boost::lexical_cast<double>(numString);
      return true;
    }
    catch(const ::boost::bad_lexical_cast & /*e*/)
    {
      return false;
    }
  }
  return false;
}

}
}
