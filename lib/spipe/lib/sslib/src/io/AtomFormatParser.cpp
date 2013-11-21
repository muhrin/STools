/*
 * StructureYamlGenerator.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "spl/io/AtomFormatParser.h"

#include <boost/algorithm/string.hpp>
#include <boost/foreach.hpp>

#include "spl/factory/SsLibElements.h"

// DEFINES /////////////////////////////////


// NAMESPACES ////////////////////////////////


namespace spl {
namespace io {

AtomFormatParser::AtomFormatParser()
{
  init();
}

AtomFormatParser::AtomFormatParser(const FormatDescription & formatDescription)
{
  init();
  setFormat(formatDescription);
}

AtomFormatParser & AtomFormatParser::operator =(const AtomFormatParser & toCopy)
{
  myDefaults = toCopy.myDefaults;
  myFormatOrder = toCopy.myFormatOrder;
  myFormatEntries = toCopy.myFormatEntries;
  return *this;
}

bool AtomFormatParser::setFormat(const FormatDescription & formatDescription)
{
  // TODO: Change this method so that it doesn't affect our state if it fails!
  myFormatOrder.clear();

  bool succeeded = true;
  BOOST_FOREACH(const ::std::string & formatString, formatDescription)
  {
    ::std::vector< ::std::string> strs;
    ::boost::split(strs, formatString, boost::is_any_of("="));
    if(strs.size() > 2)
    {
      succeeded = false;
      break;
    }
    ::boost::algorithm::trim(strs[0]);

    const FormatEntries::const_iterator it = myFormatEntries.find(strs[0]);
    if(it == myFormatEntries.end())
    {
      succeeded = false;
      break;
    }
    
    myFormatOrder.push_back(it->second);

    // Check if there is a default value
    if(strs.size() == 2)
    {
      // TODO: Complete this
    }
  }

  return succeeded;
}

void AtomFormatParser::init()
{
  addEntry("spec", factory::SPECIES);
  addEntry("radius", factory::RADIUS);
  addEntry("pos", factory::POSITION);
  addEntry("label", factory::LABEL);
}

}
}
