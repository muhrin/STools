/*
 * InfoToken.cpp
 *
 *  Created on: Dec 10, 2012
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "utility/InfoToken.h"


// NAMESPACES ////////////////////////////////

namespace stools {
namespace utility {

InfoToken::InfoToken(const ::std::string & symbol, const ::std::string & defaultFormatString):
mySymbol(symbol),
myDefaultFormatString(defaultFormatString)
{}

const ::std::string & InfoToken::getSymbol() const
{
  return mySymbol;
}

const ::std::string & InfoToken::getDefaultFormatString() const
{
  return myDefaultFormatString;
}

}
}
