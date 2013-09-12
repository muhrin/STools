/*
 * DataTableInserters.h
 *
 *
 *  Created on: Sep 12, 2013
 *      Author: Martin Uhrin
 */

#ifndef DATA_TABLE_INSERTERS_H
#define DATA_TABLE_INSERTERS_H

// INCLUDES /////////////////////////////////////////////
#include "StructurePipe.h"

#include <string>

#include <boost/filesystem/path.hpp>
#include <boost/lexical_cast.hpp>

#include <spl/common/Structure.h>
#include <spl/utility/HeterogeneousMapKey.h>

#include "SpTypes.h"
#include "utility/DataTable.h"

namespace spipe {
namespace common {
class StructureData;
}
namespace utility {
// FORWARD DECLARATIONS ////////////////////////////////////

bool
insertStructureInfoAndPotentialParams(const utility::DataTable::Key & key,
    const common::StructureData & structureData,
    const ::boost::filesystem::path & pathsRelativeTo, DataTable & table);

template< typename T>
  bool
  insertStructureProperty(const ::std::string & key,
      const ::std::string & name,
      const ::spl::utility::Key< T> & property,
      const ::spl::common::Structure & structure, DataTable & table)
  {
    const T * const value = structure.getProperty(property);
    if(!value)
      return false;

    table.insert(key, name,
        ::boost::lexical_cast< ::std::string>(*value));
    return true;
  }

}
}

#endif /* DATA_TABLE_INSERTERS_H */
