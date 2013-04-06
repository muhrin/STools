/*
 * Parsing.h
 *
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef PARSING_H
#define PARSING_H

// INCLUDES /////////////////////////////////////////////
#include <istream>

namespace sstbx {
// FORWARD DECLARATIONS ////////////////////////////////////

namespace io {

extern const ::std::string PATTERN_FLOAT;
extern const ::std::string PATTERN_RANGE;
extern const ::std::string PATTERN_RANGE_CAPTURE;

bool findNextLine(
  ::std::string & matchingLine,
  ::std::istream & inputStream,
  const ::std::string & token,
  const bool caseSensitive = true
);

bool findLastLine(
  ::std::string & matchingLine,
  ::std::istream & inputStream,
  const ::std::string & token,
  const bool caseSensitive = true
);

bool findFirstFloat(
  double & number,
  const ::std::string & line
);

::std::string getLastLine(::std::istream & is);
::std::string getLastNonEmptyLine(::std::istream & is);

}
}

#endif /* PARSING_H */
