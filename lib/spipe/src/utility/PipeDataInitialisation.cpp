/*
 * PipeDataInitialisation.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: muhrin
 */

// INCLUDES //////////////////////////////////
#include "utility/PipeDataInitialisation.h"

#include <spl/SSLibTypes.h>
#include <spl/io/CastepReader.h>
#include <spl/io/CellReaderWriter.h>
#include <spl/io/StructureReadWriteManager.h>
#include <spl/io/ResReaderWriter.h>
#include <spl/io/SslibReaderWriter.h>
#include <spl/io/XyzReaderWriter.h>

// From local
#include "common/GlobalData.h"
#include "common/SharedData.h"

// NAMESPACES ////////////////////////////////

namespace spipe {
namespace utility {

namespace sp = ::spipe;
namespace spc = ::spipe::common;
namespace ssio = ::spl::io;

void initStructureRwManDefault(ssio::StructureReadWriteManager & rwMan)
{
  rwMan.insert(::spl::makeUniquePtr(new ssio::CastepReader()));
  rwMan.insert(::spl::makeUniquePtr(new ssio::CellReaderWriter()));
  rwMan.insert(::spl::makeUniquePtr(new ssio::ResReaderWriter()));
  rwMan.insert(::spl::makeUniquePtr(new ssio::SslibReaderWriter()));
  rwMan.insert(::spl::makeUniquePtr(new ssio::XyzReaderWriter()));
  rwMan.setDefaultWriter(ssio::SslibReaderWriter::DEFAULT_EXTENSION);
}

sp::SpEngine::RunnerPtr generateRunnerInitDefault(sp::SpEngine & engine)
{
  sp::SpEngine::RunnerPtr runner = engine.createRunner();
  initGlobalDataDefault(runner->memory().global());
  return runner;
}

void initGlobalDataDefault(spc::GlobalData & global)
{
  initStructureRwManDefault(global.getStructureIo());
}

}
}
