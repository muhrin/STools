/*
 * TranscodeGeneral.h
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

#ifndef TRANSCODE_GENERAL_DETAIL_H
#define TRANSCODE_GENERAL_DETAIL_H

// INCLUDES //////////////////////////////////
#include <iomanip>
#include <sstream>
#include <string>

#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/regex.hpp>
#include <boost/tokenizer.hpp>

#include "spl/io/Parsing.h"
#include "spl/utility/Armadillo.h"

// NAMESPACES ////////////////////////////////
namespace ssy = spl::yaml;

namespace YAML {

template< typename T>
  Node
  convert< spl::yaml::TypeWrapper< std::vector< T> > >::encode(
      const typename ssy::VecAsString< T>::Type & vector)
  {
    Node node;
    BOOST_FOREACH(const T & value, *vector)
      node.push_back(value);

    return node;
  }

template< typename T>
  bool
  convert< spl::yaml::TypeWrapper< std::vector< T> > >::decode(
      const Node & node, typename ssy::VecAsString< T>::Type & vector)
  {
    if(node.IsSequence())
    {
      BOOST_FOREACH(const Node & value, node)
      {
        try
        {
          vector->push_back(value.as< T>());
        }
        catch(const YAML::TypedBadConversion< T> & /*e*/)
        {
          return false;
        }
      }
    }
    else if(node.IsScalar())
    {
      typedef boost::tokenizer< boost::char_separator< char> > Tok;
      const boost::char_separator< char> sep(" ");

      // Treat each entry in the string as a scalar node and
      // try conversion on that

      // Have to 'save' the string otherwise tokenizer doesn't like it
      const std::string tokenString(node.Scalar());
      Tok tok(tokenString, sep);
      BOOST_FOREACH(const std::string & entry, tok)
      {
        Node entryNode;
        entryNode = entry;
        try
        {
          vector->push_back(entryNode.as< T>());
        }
        catch(const YAML::TypedBadConversion< T> & /*e*/)
        {
          return false;
        }
      }
    }
    else
      return false;

    return true;
  }

template< typename T>
  Node
  convert< arma::Mat< T> >::encode(const arma::Mat< T> & rhs)
  {
    Node node;

    // TODO: Figure out a way to print without \n's from raw_print
    std::stringstream ss;
    rhs.raw_print(ss);
    node = ss.str();

    return node;
  }

template< typename T>
  bool
  convert< arma::Mat< T> >::decode(const Node & node, arma::Mat< T> & rhs)
  {
    if(!node.IsScalar())
      return false;

    std::stringstream ss(node.Scalar());
    try
    {
      ss >> rhs;
    }
    catch(const std::logic_error & /*e*/)
    {
      return false;
    }

    return true;
  }

template< typename T>
  Node
  convert< spl::utility::Range< T> >::encode(
      const spl::utility::Range< T> & rhs)
  {
    std::stringstream ss;
    if(rhs.nullSpan())
      ss << rhs.lower();
    else
      ss << rhs.lower() << "-" << rhs.upper();
    Node node;
    node = ss.str();
    return node;
  }

template< typename T>
  bool
  convert< spl::utility::Range< T> >::decode(const Node & node,
      spl::utility::Range< T> & rhs)
  {
    namespace ssio = spl::io;

    if(!node.IsScalar())
      return false;

    static const boost::regex RE_RANGE(ssio::PATTERN_RANGE_CAPTURE);

    const std::string rangeString = node.Scalar();
    boost::smatch match;
    if(::boost::regex_search(rangeString, match, RE_RANGE))
    {
      // Did we only match the first number?
      if(match[1].matched && !match[3].matched)
      { // Has single number
        const std::string lower(match[1].first, match[1].second);
        try
        {
          const T x0 = boost::lexical_cast< T>(lower);
          rhs.set(x0, x0);
          return true;
        }
        catch(const boost::bad_lexical_cast & /*e*/)
        {
        }
      }
      else if(match[1].matched && match[4].matched)
      { // Has number x0-x1
        const std::string lower(match[1].first, match[1].second);
        const std::string upper(match[4].first, match[4].second);
        try
        {
          const T x0 = boost::lexical_cast< T>(lower);
          const T x1 = boost::lexical_cast< T>(upper);
          rhs.set(x0, x1);
          return true;
        }
        catch(const boost::bad_lexical_cast & /*e*/)
        {
        }
      }
    }
    return false;
  }

}

#endif /* TRANSCODE_GENERAL_DETAIL_H */
