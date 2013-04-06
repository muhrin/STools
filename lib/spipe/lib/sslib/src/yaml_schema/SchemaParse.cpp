/*
 * SchemaParse.cpp
 *
 *  Created on: May 29, 2012
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "yaml_schema/SchemaParse.h"

#include <iostream>
#include <sstream>

#include <boost/foreach.hpp>

// NAMESPACES ////////////////////////////////

namespace sstbx {
namespace yaml_schema {

SchemaParseError & SchemaParseError::operator =(const SchemaParseError & rhs)
{
  myCode = rhs.myCode;
  myPath = rhs.myPath;
  myMessage = rhs.myMessage;
  return *this;
}

void SchemaParseError::print() const
{
  ::std::cout << myPath << ": " << myMessage << " code: " << myCode << ::std::endl;
}

SchemaParse::PathPusher::PathPusher(SchemaParse & parse, const ::std::string & path):
myParse(parse),
myPath(path)
{
  myParse.pushPath(myPath);
}

SchemaParse::PathPusher::~PathPusher()
{
  myParse.popPath();
}

bool SchemaParse::hasErrors() const
{
  return !myParseErrors.empty();
}

const SchemaParse::ParseErrors & SchemaParse::getErrors() const
{
  return myParseErrors;
}

::std::string SchemaParse::pathString() const
{
  ::std::stringstream ss;
  for(size_t i = 0; i < myParsePath.size(); ++i)
  {
    ss << myParsePath[i] << ".";
  }
  if(!myParsePath.empty())
    ss << myParsePath.back();
  return ss.str();
}

void SchemaParse::logError(const SchemaParseErrorCode::Value code)
{
  myParseErrors.push_back(SchemaParseError(code, pathString()));
}

void SchemaParse::logError(const SchemaParseErrorCode::Value code, const ::std::string & message)
{
  myParseErrors.push_back(SchemaParseError(code, pathString(), message));
}

void SchemaParse::printErrors() const
{
  BOOST_FOREACH(const SchemaParseError & error, myParseErrors)
  {
    error.print();
  }
}

void SchemaParse::pushPath(const std::string & path)
{
  myParsePath.push_back(path);
}

void SchemaParse::popPath()
{
  myParsePath.pop_back();
}

}
}


