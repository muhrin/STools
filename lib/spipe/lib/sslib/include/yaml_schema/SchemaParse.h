/*
 * SchemaParse.h
 *
 *
 *  Created on: May 29, 2012
 *      Author: Martin Uhrin
 */

#ifndef SCHEMA_PARSE_H
#define SCHEMA_PARSE_H

// INCLUDES /////////////////////////////////////////////
#include <string>
#include <vector>

// DEFINES //////////////////////////////////////////////

// FORWARD DECLARATIONS ////////////////////////////////////


namespace sstbx {
namespace yaml_schema {

struct SchemaParseErrorCode
{
  enum Value
  {
    TYPE_CONVERSION_FAILED,
    REQUIRED_VALUE_MISSING,
    NODE_TYPE_WRONG
  };
};

class SchemaParseError
{
public:
  SchemaParseError(const SchemaParseErrorCode::Value code, const ::std::string & path):
    myCode(code),
    myPath(path)
  {}

    SchemaParseError(
      const SchemaParseErrorCode::Value code,
      const ::std::string & path,
      const ::std::string & message):
    myCode(code),
    myPath(path),
    myMessage(message)
  {}

  SchemaParseError & operator =(const SchemaParseError & rhs);

  const SchemaParseErrorCode::Value & getCode() const
  { return myCode; }

  const ::std::string & getPath() const
  { return myPath; }

  const ::std::string & getMessage() const
  { return myMessage; }

  void print() const;

private:
  SchemaParseErrorCode::Value myCode;
  ::std::string myPath;
  ::std::string myMessage;
};

class SchemaParse
{
public:
  typedef ::std::vector<SchemaParseError> ParseErrors;

  void pushPath(const ::std::string & path);
  void popPath();

  bool hasErrors() const;
  const ParseErrors & getErrors() const;

  ::std::string pathString() const;

  void logError(const SchemaParseErrorCode::Value code);
  void logError(const SchemaParseErrorCode::Value code, const ::std::string & message);

  void printErrors() const;

private:
  typedef ::std::vector< ::std::string> ParsePath;

  ParsePath myParsePath;
  ParseErrors myParseErrors;
};

}
}

#endif /* SCHEMA_PARSE_H */
