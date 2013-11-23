/*
 * PipeFunctions.h
 *
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef PIPE_FUNCTIONS_H
#define PIPE_FUNCTIONS_H

// INCLUDES /////////////////////////////////////////////
#include "StructurePipe.h"

#include <string>
#include <utility>

#include <boost/concept_check.hpp>
#include <boost/filesystem/path.hpp>

#include <pipelib/pipelib.h>

#include <spl/utility/HeterogeneousMap.h>

#include "common/DataLocation.h"
#include "common/GlobalData.h"
#include "common/SharedData.h"
#include "common/StructureData.h"

// FORWARD DECLARATIONS ////////////////////////////////////

namespace spipe {
namespace common {

// TODO: Make this better! Don't want to have ObjectData<T> makes user facing code ugly!

template< typename T>
  struct ObjectData : public ::std::pair< DataLocation::Value, T *>
  {
    ObjectData()
    {
    }
    ObjectData(const typename DataLocation::Value & x, const T * const y) :
        ::std::pair< DataLocation::Value, T *>(x, y)
    {
    }
  };

template< typename T>
  ObjectData< T>
  getObject(const ::spl::utility::Key< T> & key, StructureData & strDat,
      SharedData & shared, GlobalData & global)
  {
    ObjectData< T> result;

    // First try the object from the structure data
    result.first = DataLocation::STRUCTURE;
    result.second = strDat.objectsStore.find(key);

    if(!result.second)
    {
      // Try using shared/global memory
      result = getObject(key, shared, global);
    }

    return result.second;
  }

template< typename T>
  ObjectData< T>
  getObject(::spl::utility::Key< T> & key, const SharedData & shared,
      const GlobalData & global)
  {
    ObjectData< T> result;

    // Try getting the object from shared data
    result.second = shared.objectsStore.find(key);
    if(result.first)
      result.first = DataLocation::SHARED;
    else
    {
      // Try getting the object from global data
      result.second = global.objectsStore.find(key);
      if(result.second)
        result.first = DataLocation::GLOBAL;
    }

    return result;
  }

template< typename T>
  const ObjectData< const T>
  getObjectConst(const ::spl::utility::Key< T> & key, const SharedData & shared,
      const GlobalData & global)
  {
    ObjectData< const T> result;

    // Try getting the object from shared data
    result.second = shared.objectsStore.find(key);
    if(result.first)
      result.first = DataLocation::SHARED;
    else
    {
      // Try getting the object from global data
      result.second = global.objectsStore.find(key);
      if(result.second)
        result.first = DataLocation::GLOBAL;
    }

    return result;
  }

template< typename T>
  bool
  setObject(::spl::utility::Key< T> & key, const DataLocation::Value location, const T & value,
      SharedData & shared, GlobalData & global)
  {
    if(location == DataLocation::SHARED)
    {
      shared.objectsStore[key] = value;
      return true;
    }
    else if(location == DataLocation::GLOBAL)
    {
      global.objectsStore[key] = value;
      return true;
    }

    return false;
  }

::std::string
getOutputFileStem(const SharedData & shared, const GlobalData & global);

::boost::filesystem::path
getWorkingDir(const SharedData & shared, const GlobalData & global);

::std::string
generateStructureName(const GlobalData & global);

}
}

#endif /* PIPE_FUNCTIONS_H */

