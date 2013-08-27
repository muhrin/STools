/*
 * AtomFormatParser.h
 *
 *
 *  Created on: May 29, 2012
 *      Author: Martin Uhrin
 */

#ifndef ATOM_FORMAT_PARSER_DETAIL_H
#define ATOM_FORMAT_PARSER_DETAIL_H

// INCLUDES /////////////////////////////////////////////
#include <boost/foreach.hpp>
#include <boost/scoped_ptr.hpp>

// DEFINES //////////////////////////////////////////////

// FORWARD DECLARATIONS ////////////////////////////////////

namespace spl {
namespace io {

template <typename T>
void AtomFormatParser::addEntry(const ::std::string & name, utility::Key<T> & key)
{
  myFormatEntries[name] = key.getId();
}

template <typename T>
bool AtomFormatParser::setDefault(utility::Key<T> & key, const T & value)
{
  bool found = true;
  BOOST_FOREACH(FormatEntries::const_reference entry, myFormatEntries)
  {
    if(entry.second == key.getId())
    {
      found = true;
      break;
    }
  }
  if(!found)
    return false;

  myDefaults[key] = value;
  return true;
}

template <typename T>
::boost::optional<T> AtomFormatParser::getValue(const utility::Key<T> & key, const factory::AtomsDataEntry & atomData) const
{
  ::boost::optional<T> value;
  {
    const utility::HeterogeneousMap * const map = ::boost::get<utility::HeterogeneousMap>(&atomData);
    if(map)
    {
      const T * const mapValue = map->find(key);
      if(mapValue)
        value.reset(*mapValue);
    }
    else
    {
      const ::std::vector< ::std::string> * const vector = ::boost::get< ::std::vector< ::std::string> >(&atomData);
      if(vector)
      {
        // Check if we know this key type
        size_t i;
        for(i = 0; i < myFormatOrder.size(); ++i)
        {
          if(myFormatOrder[i] == key.getId())
            break;
        }
        if(i != myFormatOrder.size() && i < vector->size())
        {
          const ::std::string valueString = (*vector)[i];
          // TODO: Parse string and set to value
        }
      }
    }
  }
  
  if(!value)
  {
    const T * const mapValue = myDefaults.find(key);
    if(mapValue)
      value.reset(*mapValue);
  }

  return value;
}

} // namespace io
} // namespace spl

#endif /* ATOM_FORMAT_PARSER_DETAIL_H */
