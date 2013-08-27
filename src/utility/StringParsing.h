/*
 * StringParsing.h
 *
 *
 *  Created on: Nov 19, 2012
 *      Author: Martin Uhrin
 */

#ifndef STRING_PARSING_H
#define STRING_PARSING_H

// INCLUDES /////////////////////////////////////////////
#include <string>

// FORWARD DECLARES ////////////////////////////////

namespace stools {
namespace utility {

void replaceControlSequences(::std::string & str);
::std::string replaceControlSequencesCopy(const ::std::string & str);

void removeControlSequences(::std::string & str);
::std::string removeControlSequences(const ::std::string & str);

void removeVerticalPositioningSequences(::std::string & str);
::std::string removeVerticalPositioningSequencesCopy(const ::std::string & str);

}
}


#endif /* STRING_PARSING_H */
