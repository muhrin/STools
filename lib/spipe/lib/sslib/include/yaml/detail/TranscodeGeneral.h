/*
 * TranscodeGeneral.h
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

#ifndef TRANSCODE_GENERAL_DETAIL_H
#define TRANSCODE_GENERAL_DETAIL_H

// INCLUDES //////////////////////////////////
#include <sstream>
#include <string>

#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/tokenizer.hpp>

#include "io/Parsing.h"

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

template <unsigned int size>
Node convert< ::arma::vec::fixed<size> >::encode(const ::arma::vec::fixed<size> & rhs)
{
  Node node;
  ::std::stringstream ss;
  ss << ::std::setprecision(12);
  for(size_t i = 0; i < rhs.size() - 1; ++i)
    ss << rhs(i) << " ";
  ss << rhs(rhs.size() - 1); // Do the last one separately so we don't have a trailing space
  node = ss.str();
  return node;
}

template <unsigned int size>
bool convert< ::arma::vec::fixed<size> >::decode(const Node & node, ::arma::vec::fixed<size> & rhs)
{
  typedef ssy::VectorAsString<double> DoublesVec;
  // Maybe it is a string separated by spaces
  DoublesVec doublesVec;

  try
  {
    doublesVec = node.as<DoublesVec>();
  }
  catch(const YAML::TypedBadConversion<DoublesVec> & /*e*/)
  {
    return false;
  }

  if(doublesVec->size() != size)
    return false; // Expecting 3 coordinates

  // Copy over values
  for(size_t i = 0; i < size; ++i)
    rhs(i) = (*doublesVec)[i];

  return true;
}

template <typename T>
Node convert< ::sstbx::utility::Range<T> >::encode(const ::sstbx::utility::Range<T> & rhs)
{
  ::std::stringstream ss;
  if(rhs.nullSpan())
    ss << rhs.lower();
  else
    ss << rhs.lower() << "-" << rhs.upper();
  Node node;
  node = ss.str();
  return node;
}

template <typename T>
bool convert< ::sstbx::utility::Range<T> >::decode(const Node & node, ::sstbx::utility::Range<T> & rhs)
{
  namespace ssio = ::sstbx::io;

  if(!node.IsScalar())
    return false;

  static const ::boost::regex RE_RANGE(ssio::PATTERN_RANGE_CAPTURE);
  
  const ::std::string rangeString = node.Scalar();
  ::boost::smatch match;
  if(::boost::regex_search(rangeString, match, RE_RANGE))
  {
    // Did we only match the first number?
    if(match[1].matched && !match[3].matched)
    { // Has single number
      const ::std::string lower(match[1].first, match[1].second);
      try
      {
        const T x0 = ::boost::lexical_cast<T>(lower);
        rhs.set(x0, x0);
        return true;
      }
      catch(const ::boost::bad_lexical_cast & /*e*/)
      {}
    }
    else if(match[1].matched && match[4].matched)
    { // Has number x0-x1
      const ::std::string lower(match[1].first, match[1].second);
      const ::std::string upper(match[4].first, match[4].second);
      try
      {
        const T x0 = ::boost::lexical_cast<T>(lower);
        const T x1 = ::boost::lexical_cast<T>(upper);
        rhs.set(x0, x1);
        return true;
      }
      catch(const ::boost::bad_lexical_cast & /*e*/)
      {}
    }
  }
  return false;
}

}



#endif /* TRANSCODE_GENERAL_DETAIL_H */
