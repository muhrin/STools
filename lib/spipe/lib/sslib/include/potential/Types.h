/*
 * Types.h
 *
 *  Created on: Aug 30, 2012
 *      Author: Martin Uhrin
 */

#ifndef POTENTIAL__TYPES_H
#define POTENTIAL__TYPES_H

// INCLUDES ////////////
#include "SSLib.h"

namespace sstbx {
namespace potential {

// FORWARD DECLARES ////////////
class IGeomOptimiser;
class IPotential;

typedef UniquePtr<IPotential>::Type IPotentialPtr;
typedef UniquePtr<IGeomOptimiser>::Type IGeomOptimiserPtr;

}
}

#endif /* POTENTIAL__TYPES_H */
