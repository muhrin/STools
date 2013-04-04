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

bool findFirstLine(
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

}
}

#endif /* PARSING_H */
