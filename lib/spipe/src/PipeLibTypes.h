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
#include <pipelib/pipelib.h>

// FORWARD DECLARES //////////////
namespace spipe {
namespace common {
class GlobalData;
class SharedData;
class StructureData;
}
}

namespace spipe {

// TYPEDEFS ///////////////////////

typedef common::StructureData StructureDataType;
typedef common::SharedData SharedDataType;
typedef common::GlobalData GlobalDataType;

// Pipe blocks
typedef pipelib::Block< StructureDataType, SharedDataType, GlobalDataType> Block;
typedef pipelib::Block< StructureDataType, SharedDataType, GlobalDataType>::HandleType BlockHandle;
typedef pipelib::Barrier< StructureDataType, SharedDataType, GlobalDataType> Barrier;
typedef pipelib::FinishedSink< StructureDataType> FinishedSink;
typedef FinishedSink::PipeUniquePtr StructureDataUniquePtr;
typedef pipelib::PipeBlock< StructureDataType, SharedDataType, GlobalDataType> PipeBlock;
typedef pipelib::StartBlock< StructureDataType, SharedDataType, GlobalDataType> StartBlock;

// Pipe engine
typedef pipelib::PipeEngine< StructureDataType, SharedDataType, GlobalDataType> Engine;
typedef pipelib::BoostThreadEngine< StructureDataType, SharedDataType, GlobalDataType> BoostThreadEngine;
typedef pipelib::SerialEngine< StructureDataType, SharedDataType, GlobalDataType> SerialEngine;
typedef pipelib::EngineSetup< StructureDataType, SharedDataType, GlobalDataType> EngineSetup;
typedef pipelib::EngineAccess< StructureDataType, SharedDataType, GlobalDataType> EngineAccess;
typedef pipelib::SimpleBarrier< StructureDataType, SharedDataType, GlobalDataType> SimpleBarrier;

// Event
typedef pipelib::event::PipeEngineListener< EngineAccess> EngineAccessListener;
}

#endif /* PIPE_LIB_TYPES_H */
