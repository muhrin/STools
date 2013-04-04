/*
 * PipeDataInitialisation.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: muhrin
 */

// INCLUDES //////////////////////////////////
#include "utility/PipeDataInitialisation.h"

// From SSTbx
#include <SSLibTypes.h>
#include <io/CastepReader.h>
#include <io/CellReaderWriter.h>
#include <io/StructureReadWriteManager.h>
#include <io/ResReaderWriter.h>
#include <io/SslibReaderWriter.h>
#include <io/XyzReaderWriter.h>

// From local
#include "common/GlobalData.h"
#include "common/SharedData.h"

// NAMESPACES ////////////////////////////////

namespace spipe {
namespace utility {

namespace sp = ::spipe;
namespace spc = ::spipe::common;
namespace ssio = ::sstbx::io;

void initStructureRwManDefault(ssio::StructureReadWriteManager & rwMan)
{
  rwMan.insert(::sstbx::makeUniquePtr(new ssio::CastepReader()));
  rwMan.insert(::sstbx::makeUniquePtr(new ssio::CellReaderWriter()));
  rwMan.insert(::sstbx::makeUniquePtr(new ssio::ResReaderWriter()));
  rwMan.insert(::sstbx::makeUniquePtr(new ssio::SslibReaderWriter()));
  rwMan.insert(::sstbx::makeUniquePtr(new ssio::XyzReaderWriter()));
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
