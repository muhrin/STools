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
class SharedData;
class StructureData;
}
}


namespace spipe {

// TYPEDEFS ///////////////////////

typedef common::StructureData StructureDataTyp;
typedef common::SharedData SharedDataTyp;

// Pipe blocks
typedef pipelib::Block<StructureDataTyp, SharedDataTyp, SharedDataTyp>        SpBlock;
typedef pipelib::Barrier<StructureDataTyp, SharedDataTyp, SharedDataTyp>      SpBarrier;
typedef pipelib::FinishedSink<StructureDataTyp>                               SpFinishedSink;
typedef SpFinishedSink::PipelineDataPtr                                       SpStructureDataPtr;
typedef pipelib::PipeBlock<StructureDataTyp, SharedDataTyp, SharedDataTyp>    SpPipeBlock;
typedef pipelib::StartBlock<StructureDataTyp, SharedDataTyp, SharedDataTyp>   SpStartBlock;

// Pipe engine and runner
typedef pipelib::PipeRunner<StructureDataTyp, SharedDataTyp, SharedDataTyp>   SpRunner;
typedef pipelib::SingleThreadedEngine<StructureDataTyp, SharedDataTyp, SharedDataTyp> SpSingleThreadedEngine;
typedef pipelib::SingleThreadedRunner<StructureDataTyp, SharedDataTyp, SharedDataTyp> SpSingleThreadedRunner;
typedef pipelib::RunnerSetup<StructureDataTyp, SharedDataTyp, SharedDataTyp>  SpRunnerSetup;
typedef pipelib::SimpleBarrier<StructureDataTyp, SharedDataTyp, SharedDataTyp> SpSimpleBarrier;
typedef SpRunnerSetup::ChildRunnerPtr                                         SpChildRunnerPtr;
typedef pipelib::RunnerAccess<StructureDataTyp, SharedDataTyp, SharedDataTyp> SpRunnerAccess;
typedef pipelib::StartBlock<StructureDataTyp, SharedDataTyp, SharedDataTyp>   SpStartBlockTyp;
typedef pipelib::MemoryAccess<SharedDataTyp, SharedDataTyp>                   MemoryAccessType;

typedef pipelib::PipelineDataHandle                                           StructureDataHandle;

// Event
typedef pipelib::event::PipeRunnerListener<SpRunner>                          SpRunnerListener;
}

#endif /* PIPE_LIB_TYPES_H */
