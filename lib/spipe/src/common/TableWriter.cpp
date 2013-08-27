/*
 * DataTable.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "spl/common/DataTable.h"



// NAMESPACES ////////////////////////////////

namespace spipe
{
namespace common
{

DataTable::Column::Column(const ::std::string & name):
myName(name),
myIndex(COL_UNDEFINED)
{}

}
}
