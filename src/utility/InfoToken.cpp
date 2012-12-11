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

InfoToken::InfoToken(const ::std::string & symbol):
mySymbol(symbol)
{}

const ::std::string & InfoToken::getSymbol() const
{
  return mySymbol;
}

}
}
