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

#include <utility>

#include <boost/concept_check.hpp>

#include <pipelib/IPipeline.h>

#include <utility/HeterogeneousMap.h>

#include "common/DataLocation.h"
#include "common/SharedData.h"
#include "common/StructureData.h"

// FORWARD DECLARATIONS ////////////////////////////////////

namespace spipe
{
namespace common
{

static const ::std::string NAME_DELIMITER("_");

// TODO: Make this better! Don't want to have ObjectData<T> makes user facing code ugly!

template <typename T>
struct ObjectData : public ::std::pair<DataLocation::Value, T *>
{

  ObjectData() {}
  ObjectData(const DataLocation::Value & x, const T * const y) : first(x), second(y) {}
};

template <typename T>
ObjectData<T> getObject(const ::sstbx::utility::Key<T> & key, ::spipe::StructureDataTyp & strDat, ::spipe::SpPipelineTyp & pipe)
{
  ObjectData<T> result;

  // First try the object from the structure data
  result.first = DataLocation::STRUCTURE;
  result.second = strDat.objectsStore.find(key);

  if(!result.second)
  {
    // Try using shared/global memory
    result = getObject(key, pipe);
  }

  return object;
}

template <typename T>
ObjectData<T> getObject(const ::sstbx::utility::Key<T> & key, ::spipe::SpPipelineTyp & pipe)
{
  ObjectData<T> result;

  // Try getting the object from shared data
  result.second = pipe.getSharedData().objectsStore.find(key);
  if(result.first)
    result.first  = DataLocation::SHARED;    
  else
  {
    // Try getting the object from global data
    result.second = pipe.getGlobalData().objectsStore.find(key);
    if(result.second)
      result.first  = DataLocation::GLOBAL;
  }

  return result;
}

template <typename T>
const ObjectData<const T> getObjectConst(const ::sstbx::utility::Key<T> & key, const ::spipe::SpPipelineTyp & pipe)
{
  ObjectData<const T> result;

  // Try getting the object from shared data
  result.second = pipe.getSharedData().objectsStore.find(key);
  if(result.first)
    result.first  = DataLocation::SHARED;    
  else
  {
    // Try getting the object from global data
    result.second = pipe.getGlobalData().objectsStore.find(key);
    if(result.second)
      result.first  = DataLocation::GLOBAL;
  }

  return result;
}

template <typename T>
bool setObject(
  const ::sstbx::utility::Key<T> & key,
  const DataLocation::Value location,
  const T & value,
  ::spipe::SpPipelineTyp & pipe)
{
  if(location == DataLocation::SHARED)
  {
    pipe.getSharedData().objectsStore.insert(key, value);
    return true;
  }
  else if(location == DataLocation::GLOBAL)
  {
    pipe.getGlobalData().objectsStore.insert(key, value);
    return true;
  }

  return false;
}



}
}

#endif /* PIPE_FUNCTIONS_H */

