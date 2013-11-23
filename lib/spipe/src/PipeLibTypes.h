/*
 * PipeLibTypes.h
 *
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef PIPE_LIB_TYPES_H
#define PIPE_LIB_TYPES_H

// INCLUDES /////////////
#include <boost/filesystem/path.hpp>

#include <pipelib/pipelib.h>

#include <spl/SSLibTypes.h>

#include "common/Assert.h"
#include "common/PipeFunctions.h"

// FORWARD DECLARES //////////////
namespace spipe {
namespace common {
class GlobalData;
class SharedData;
class StructureData;
}
}

namespace spipe {

template< typename T>
  struct UniquePtr
  {
    typedef typename spl::UniquePtr< T>::Type Type;
  private:
    UniquePtr()
    {
    }
  };

// TYPEDEFS ///////////////////////

typedef common::StructureData StructureDataType;
typedef common::SharedData SharedDataType;
typedef common::GlobalData GlobalDataType;

template< class Base>
  class SPipeBlock : public Base
  {
  protected:
    SPipeBlock() :
        pipelib::Block< StructureDataType, SharedDataType, GlobalDataType>(
            "SPipe Block")
    {
    }
    ::boost::filesystem::path
    workingDir() const
    {
      SPIPE_ASSERT(this->getEngine());
      return common::getWorkingDir(this->getEngine()->sharedData(),
          this->getEngine()->globalData());
    }
    ::boost::filesystem::path
    outputFileStemPath() const
    {
      SPIPE_ASSERT(this->getEngine());
      return workingDir()
          / common::getOutputFileStem(this->getEngine()->sharedData(),
              this->getEngine()->globalData());
    }
  };

// Pipe blocks
typedef pipelib::Block< StructureDataType, SharedDataType, GlobalDataType> Block;
typedef pipelib::Block< StructureDataType, SharedDataType, GlobalDataType>::HandleType BlockHandle;
typedef pipelib::FinishedSink< StructureDataType> FinishedSink;
typedef FinishedSink::PipeUniquePtr StructureDataUniquePtr;
typedef SPipeBlock<
    pipelib::PipeBlock< StructureDataType, SharedDataType, GlobalDataType> > PipeBlock;
typedef SPipeBlock<
    pipelib::StartBlock< StructureDataType, SharedDataType, GlobalDataType> > StartBlock;
typedef SPipeBlock<
    pipelib::Barrier< StructureDataType, SharedDataType, GlobalDataType> > Barrier;

// Pipe engine
typedef pipelib::PipeEngine< StructureDataType, SharedDataType, GlobalDataType> Engine;
typedef pipelib::BoostThreadEngine< StructureDataType, SharedDataType,
    GlobalDataType> BoostThreadEngine;
typedef pipelib::SerialEngine< StructureDataType, SharedDataType, GlobalDataType> SerialEngine;
typedef pipelib::EngineSetup< StructureDataType, SharedDataType, GlobalDataType> EngineSetup;
typedef pipelib::EngineAccess< StructureDataType, SharedDataType, GlobalDataType> EngineAccess;
typedef pipelib::SimpleBarrier< StructureDataType, SharedDataType,
    GlobalDataType> SimpleBarrier;

// Event
typedef pipelib::event::PipeEngineListener< EngineAccess> EngineAccessListener;
}

#endif /* PIPE_LIB_TYPES_H */
