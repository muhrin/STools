/*
 * PipeDataInitialisation.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: muhrin
 */

// INCLUDES //////////////////////////////////
#include "PipeDataInitialisation.h"

// From SSTbx
#include <SSLibTypes.h>
#include <io/StructureReadWriteManager.h>
#include <io/ResReaderWriter.h>
#include <io/SslibReaderWriter.h>

// From local
#include "common/GlobalData.h"
#include "common/SharedData.h"

// NAMESPACES ////////////////////////////////

namespace spipe {
namespace common {

namespace sp = ::spipe;
namespace ssio = ::sstbx::io;

void initStructureRwManDefault(ssio::StructureReadWriteManager & rwMan)
{
  rwMan.insert(::sstbx::makeUniquePtr(new ssio::ResReaderWriter()));
  rwMan.insert(::sstbx::makeUniquePtr(new ssio::SslibReaderWriter()));
  rwMan.setDefaultWriter("sslib");
}

sp::SpEngine::RunnerPtr generateRunnerInitDefault(sp::SpEngine & engine)
{
  sp::SpEngine::RunnerPtr runner = engine.createRunner();
  initGlobalDataDefault(runner->memory().global());
  return runner;
}

void initGlobalDataDefault(GlobalData & global)
{
  initStructureRwManDefault(global.getStructureIo());
}

}
}
