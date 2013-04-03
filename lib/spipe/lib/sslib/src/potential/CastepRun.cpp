/*
 * CastepGeomOptRun.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "potential/CastepRun.h"

#include <sstream>

#include <boost/filesystem.hpp>

#include "common/AtomSpeciesDatabase.h"
#include "common/Structure.h"
#include "io/BoostFilesystem.h"
#include "io/CastepReader.h"
#include "io/CellReaderWriter.h"
#include "io/Parsing.h"
#include "os/Process.h"

// NAMESPACES ////////////////////////////////
namespace sstbx {
namespace potential {

namespace fs = ::boost::filesystem;

CastepRun::CastepRun(
  const ::std::string & seed,
  const io::CellReaderWriter & cellReaderWriter,
  const io::CastepReader & castepReader
):
myCellFile(seed + ".cell"),
myParamFile(seed + ".param"),
myCastepFile(seed + ".castep"),
myCellReaderWriter(cellReaderWriter),
myCastepReader(castepReader)
{}

CastepRun::~CastepRun()
{
  closeAllStreams();
}

const fs::path & CastepRun::getParamFile() const
{
  return myParamFile;
}

const fs::path & CastepRun::getCellFile() const
{
  return myCellFile;
}

const fs::path & CastepRun::getCastepFile() const
{
  return myCastepFile;
}

CastepRunResult::Value CastepRun::openCellFile(fs::ofstream * * ofstream)
{
  // If the file is already open close it and delete it
  if(myCellFileStream.is_open())
    myCellFileStream.close();

  if(fs::exists(myCellFile))
    fs::remove_all(myCellFile);

  myCellFileStream.open(myCellFile);
  
  if(ofstream)
    *ofstream = &myCellFileStream;

  return CastepRunResult::SUCCESS;
}

CastepRunResult::Value CastepRun::openCastepFile(fs::ifstream * * ifstream)
{
  if(!fs::exists(myCastepFile))
    return CastepRunResult::OUTPUT_NOT_FOUND;

  if(!myCastepFileStream.is_open())
    myCastepFileStream.open(myCastepFile);
  else
  { // Move the file to the beginning
    myCastepFileStream.clear(); // Clear the EoF flag
    myCastepFileStream.seekg(0, myCastepFileStream.beg);
  }

  if(ifstream)
    *ifstream = &myCastepFileStream;

  return CastepRunResult::SUCCESS;
}

void CastepRun::closeAllStreams()
{
  if(myCastepFileStream.is_open())
    myCastepFileStream.close();
  if(myCellFileStream.is_open())
    myCellFileStream.close();
}

CastepRunResult::Value CastepRun::runCastep(const fs::path & castepExe)
{
  // Make sure to close all streams so we don't end up in a conflict
  closeAllStreams();

  if(!fs::exists(myCellFile) || !fs::exists(myParamFile))
    return CastepRunResult::INPUT_NOT_FOUND;

  const ::std::vector< ::std::string> args(1, io::stemString(myCellFile));
  if(os::runBlocking(castepExe, args) != 0)
    return CastepRunResult::FAILED_TO_RUN;

  return CastepRunResult::SUCCESS;
}

CastepRunResult::Value CastepRun::updateStructureFromOutput(
  common::Structure & structure,
  const common::AtomSpeciesDatabase & speciesDb
)
{
  if(!fs::exists(myCastepFile))
    return CastepRunResult::OUTPUT_NOT_FOUND;

  CastepRunResult::Value result = openCastepFile();
  if(result != CastepRunResult::SUCCESS)
    return result;

  common::types::StructurePtr newStructure = myCastepReader.readStructure(myCastepFileStream, speciesDb);
  if(!newStructure.get())
    return CastepRunResult::FAILED_TO_READ_STRUCTURE;

  structure.clearAtoms();

  // Copy over the unit cell
  structure.setUnitCell(makeUniquePtr(new common::UnitCell(*newStructure->getUnitCell())));

  // Copy over the atoms
  for(size_t i = 0; i < newStructure->getNumAtoms(); ++i)
    structure.newAtom(newStructure->getAtom(i));

  return CastepRunResult::SUCCESS;
}

CastepRunResult::Value CastepRun::deleteAllOutput()
{
  closeAllStreams();

  // TODO: Remove all other files
  fs::remove_all(myCastepFile);

  return CastepRunResult::SUCCESS;
}

CastepRunResult::Value CastepRun::deleteAllFiles()
{
  deleteAllOutput();
  fs::remove_all(myCellFile);
  fs::remove_all(myParamFile);

  return CastepRunResult::SUCCESS;
}

}
}

