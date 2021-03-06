/*
 * StringParsing.cpp
 *
 *  Created on: Dec 10, 2012
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "utility/StringParsing.h"

#include <vector>

#include <boost/algorithm/string.hpp>
#include <boost/foreach.hpp>

// NAMESPACES ////////////////////////////////

namespace stools {
namespace utility {

typedef ::std::pair< ::std::string, ::std::string> ControlSeqReplace;

static const ControlSeqReplace controlSeqReplace[] =
{
  ControlSeqReplace("\\a", "\a"), // alert bell
  ControlSeqReplace("\\b", "\b"), // backspace
  ControlSeqReplace("\\t", "\t"), // horizontal tab
  ControlSeqReplace("\\n", "\n"), // new line (line feed)
  ControlSeqReplace("\\v", "\v"), // vertical tab
  ControlSeqReplace("\\f", "\f"), // form feed
  ControlSeqReplace("\\r", "\r")  // return carriage
};

void replaceControlSequences(::std::string & str)
{
  BOOST_FOREACH(const ControlSeqReplace & seq, controlSeqReplace)
    ::boost::replace_all(str, seq.first, seq.second);
}

::std::string replaceControlSequencesCopy(const ::std::string & str)
{
  ::std::string result = str;
  replaceControlSequences(result);
  return result;
}

void removeControlSequences(::std::string & str)
{
  BOOST_FOREACH(const ControlSeqReplace & seq, controlSeqReplace)
    ::boost::replace_all(str, seq.second, "");
}

::std::string removeControlSequences(const ::std::string & str)
{
  ::std::string result = str;
  removeControlSequences(result);
  return result;
}

void removeVerticalPositioningSequences(::std::string & str)
{
  ::boost::replace_all(str, "\n", "");
  ::boost::replace_all(str, "\v", "");
  ::boost::replace_all(str, "\f", "");
  ::boost::replace_all(str, "\r", "");
}

::std::string removeVerticalPositioningSequencesCopy(const ::std::string & str)
{
  ::std::string result = str;
  removeVerticalPositioningSequences(result);
  return result;
}

}
}
