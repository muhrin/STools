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
#include <spl/io/SplReaderWriter.h>
#include <spl/io/XyzReaderWriter.h>

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
  rwMan.insert(::spl::makeUniquePtr(new ssio::SplReaderWriter()));
  rwMan.insert(::spl::makeUniquePtr(new ssio::XyzReaderWriter()));
  rwMan.setDefaultWriter(ssio::SplReaderWriter::DEFAULT_EXTENSION);
}


}
}
