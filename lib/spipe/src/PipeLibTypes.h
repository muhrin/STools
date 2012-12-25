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
typedef pipelib::Block<StructureDataType, SharedDataType, GlobalDataType>        SpBlock;
typedef pipelib::Barrier<StructureDataType, SharedDataType, GlobalDataType>      SpBarrier;
typedef pipelib::FinishedSink<StructureDataType>                               SpFinishedSink;
typedef SpFinishedSink::PipelineDataPtr                                       SpStructureDataPtr;
typedef pipelib::PipeBlock<StructureDataType, SharedDataType, GlobalDataType>    SpPipeBlock;
typedef pipelib::StartBlock<StructureDataType, SharedDataType, GlobalDataType>   SpStartBlock;

// Pipe engine and runner
typedef pipelib::PipeRunner<StructureDataType, SharedDataType, GlobalDataType>   SpRunner;
typedef pipelib::SingleThreadedEngine<StructureDataType, SharedDataType, GlobalDataType> SpSingleThreadedEngine;
typedef pipelib::SingleThreadedRunner<StructureDataType, SharedDataType, GlobalDataType> SpSingleThreadedRunner;
typedef pipelib::RunnerSetup<StructureDataType, SharedDataType, GlobalDataType>  SpRunnerSetup;
typedef pipelib::SimpleBarrier<StructureDataType, SharedDataType, GlobalDataType> SpSimpleBarrier;
typedef SpRunnerSetup::ChildRunnerPtr                                         SpChildRunnerPtr;
typedef pipelib::RunnerAccess<StructureDataType, SharedDataType, GlobalDataType> SpRunnerAccess;
typedef pipelib::StartBlock<StructureDataType, SharedDataType, GlobalDataType>   SpStartBlockTyp;
typedef pipelib::MemoryAccess<SharedDataType, GlobalDataType>                   MemoryAccessType;

typedef pipelib::PipelineDataHandle                                           StructureDataHandle;

// Event
typedef pipelib::event::PipeRunnerListener<SpRunner>                          SpRunnerListener;
}

#endif /* PIPE_LIB_TYPES_H */
