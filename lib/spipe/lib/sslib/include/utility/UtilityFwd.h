/*
 * UtilityFwd.h
 *
 *  Created on: Aug 30, 2012
 *      Author: Martin Uhrin
 */

#ifndef UTILITY_FWD_H
#define UTILITY_FWD_H

// INCLUDES ////////////
#include "SSLib.h"

namespace sstbx {
namespace utility {

// FORWARD DECLARES ////////////
class IStructureComparator;


typedef UniquePtr<IStructureComparator>::Type IStructureComparatorPtr;


}
}

#endif /* UTILITY_FWD_H */
