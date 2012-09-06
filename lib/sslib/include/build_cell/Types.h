/*
 * Types.h
 *
 *  Created on: Aug 30, 2012
 *      Author: Martin Uhrin
 */

#ifndef BUILD_CELL__TYPES_H
#define BUILD_CELL__TYPES_H

// INCLUDES ////////////
#include <boost/shared_ptr.hpp>


namespace sstbx {
namespace build_cell {

// FORWARD DECLARES ////////////
class IUnitCellBlueprint;

typedef ::boost::shared_ptr<const IUnitCellBlueprint> ConstUnitCellBlueprintPtr;

}
}

#endif /* BUILD_CELL__TYPES_H */
