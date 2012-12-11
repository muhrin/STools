/*
 * CustomTokens.h
 *
 *
 *  Created on: Nov 19, 2012
 *      Author: Martin Uhrin
 */

#ifndef CUSTOM_TOKENS_H
#define CUSTOM_TOKENS_H

// INCLUDES /////////////////////////////////////////////
#include "utility/InfoToken.h"

// FORWARD DECLARES ////////////////////////////////
namespace sstbx {
namespace common {
class Structure;
}
}

namespace stools {
namespace utility {
namespace functions {
::boost::optional< ::std::string> getName(const ::sstbx::common::Structure & structure);
::boost::optional<double> getVolume(const ::sstbx::common::Structure & structure);
::boost::optional<double> getEnergyPerAtom(const ::sstbx::common::Structure & structure);
::boost::optional<unsigned int> getNumAtoms(const ::sstbx::common::Structure & structure);
}

}
}


#endif /* CUSTOM_TOKENS_H */
