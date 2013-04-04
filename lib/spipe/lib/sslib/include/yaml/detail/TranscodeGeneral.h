/*
 * TranscodeGeneral.h
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

#ifndef TRANSCODE_GENERAL_DETAIL_H
#define TRANSCODE_GENERAL_DETAIL_H

// INCLUDES //////////////////////////////////
#include <string>

#include <boost/foreach.hpp>
#include <boost/tokenizer.hpp>

// NAMESPACES ////////////////////////////////
namespace ssy = ::sstbx::yaml;

namespace YAML {

template <typename T>
Node convert< ssy::VectorAsString<T> >::encode(const ssy::VectorAsString<T> & vector)
{
  Node node;
  BOOST_FOREACH(const T & value, *vector)
  {
    node.push_back(value);
  }
  return node;
}

template <typename T>
bool convert<ssy::VectorAsString<T> >::decode(const Node & node, ssy::VectorAsString<T> & vector)
{
  if(node.IsSequence())
  {
    BOOST_FOREACH(const Node & value, node)
    {
      try
      {
        vector->push_back(value.as<T>());
      }
      catch(const YAML::TypedBadConversion<T> & /*e*/)
      {
        return false;
      }
    }
  }
  else if(node.IsScalar())
  {
    typedef boost::tokenizer<boost::char_separator<char> > Tok;
    const boost::char_separator<char> sep(" ");

    // Treat each entry in the string as a scalar node and
    // try conversion on that

    // Have to 'save' the string otherwise tokenizer doesn't like it
    const ::std::string tokenString(node.Scalar());
    Tok tok(tokenString, sep);
    BOOST_FOREACH(const ::std::string & entry, tok)
    {
      Node entryNode;
      entryNode = entry;
      try
      {
        vector->push_back(entryNode.as<T>());
      }
      catch(const YAML::TypedBadConversion<T> & /*e*/)
      {
        return false;
      }
    }
  }
  else
    return false;

  return true;
}

}



#endif /* TRANSCODE_GENERAL_DETAIL_H */
