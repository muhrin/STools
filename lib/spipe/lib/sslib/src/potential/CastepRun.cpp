/*
 * CastepGeomOptRun.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "potential/CastepRun.h"

#include <sstream>

#include <boost/algorithm/string.hpp> // TODO: REMOVE
#include <boost/filesystem.hpp>

#include "common/AtomSpeciesDatabase.h"
#include "common/Structure.h"
#include "io/CastepReader.h"
#include "io/CellReaderWriter.h"
#include "io/Parsing.h"
#include "os/Process.h"

#include "potential/PotentialData.h" // TODO: REMOVE

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
  if(!myCellFileStream.is_open())
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
  if(!fs::exists(myCellFile) || !fs::exists(myParamFile))
    return CastepRunResult::INPUT_NOT_FOUND;

  // Make sure to close all streams so we don't end up in a conflict
  closeAllStreams();

  const ::std::vector< ::std::string> args(1, myCellFile.filename().string());
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

const int CastepGeomOptRun::MAX_RELAX_ATTEMPTS = 40;

CastepGeomOptRun::CastepGeomOptRun(
  const ::std::string & originalSeed,
  const ::std::string & newSeed,
  const bool keepIntermediates,
  const io::CellReaderWriter & cellReaderWriter,
  const io::CastepReader & castepReader
):
myOrigCellFile(originalSeed + ".cell"),
myOrigParamFile(originalSeed + ".param"),
myCastepRun(newSeed, cellReaderWriter, castepReader),
myKeepIntermediates(keepIntermediates),
myCellReaderWriter(cellReaderWriter),
myCastepReader(castepReader)
{}

CastepGeomOptRun::~CastepGeomOptRun()
{
  if(!myKeepIntermediates)
    myCastepRun.deleteAllFiles();
}

OptimisationOutcome CastepGeomOptRun::runFullRelax(
  common::Structure & structure,
  PotentialData & data,
  const fs::path & castepExe,
  const common::AtomSpeciesDatabase & speciesDb,
  const int numRelaxations
)
{
  if(!fs::exists(myOrigParamFile))
  {
    ::std::stringstream ss;
    ss << "Castep input file " << myOrigParamFile << " not found.";
    return OptimisationOutcome::failure(OptimisationError::INTERNAL_ERROR, ss.str());
  }

  if(!copyParamFile())
  {
    ::std::stringstream ss;
    ss << "Failed to copy " << myOrigParamFile << " to " << myCastepRun.getParamFile() << ".";
    return OptimisationOutcome::failure(OptimisationError::INTERNAL_ERROR, ss.str());
  }
  
   const OptimisationOutcome copyResult = makeCellCopy(structure, speciesDb);
   if(!copyResult.isSuccess())
     return copyResult;
  
  OptimisationOutcome relaxOutcome;
  int successfulRelaxations = 0;
  int i;
  for(i = 0; successfulRelaxations < numRelaxations && i < MAX_RELAX_ATTEMPTS; ++i)
  {
    relaxOutcome = doRelaxation(structure, speciesDb, castepExe);
    if(!relaxOutcome.isSuccess())
      return relaxOutcome;

    // Keep relaxing until we get somewhere
    if(optimistaionSucceeded())
      ++successfulRelaxations;
  }
  // How did we exit?
  if(i == MAX_RELAX_ATTEMPTS)
    return OptimisationOutcome::failure(OptimisationError::FAILED_TO_CONVERGE);

  // Finally update the optimistaion information
  updateOptimisationInfo(structure, data, speciesDb);

  return OptimisationOutcome::success();
}


bool CastepGeomOptRun::copyParamFile() const
{
  SSLIB_ASSERT(fs::exists(myOrigParamFile));

  // Copy over the .param file to the new seed name
  try
  {
    fs::copy_file(myOrigParamFile, myCastepRun.getParamFile(),  fs::copy_option::overwrite_if_exists);
  }
  catch(const fs::filesystem_error & /*e*/)
  {
    return false;
  }
  return true;
}

OptimisationOutcome CastepGeomOptRun::makeCellCopy(
  common::Structure & structure,
  const common::AtomSpeciesDatabase & speciesDb
)
{
  fs::ofstream * newCellFileStream;
  if(myCastepRun.openCellFile(&newCellFileStream) == CastepRunResult::SUCCESS)
  {  
    myCellReaderWriter.writeStructure(*newCellFileStream, structure, speciesDb);

    // Now copy over the original contents
    if(fs::exists(myOrigCellFile))
    {
      fs::ifstream origCellStream(myOrigCellFile);
      if(origCellStream.is_open())
      {
        *newCellFileStream << ::std::endl << "#==ORIGINAL CONTENTS==" << ::std::endl;
        *newCellFileStream << origCellStream.rdbuf();
        origCellStream.close();
      }
      else
      {
        ::std::stringstream ss;
        ss << "Failed to open " << myOrigCellFile << " for reading.";
        return OptimisationOutcome::failure(OptimisationError::INTERNAL_ERROR, ss.str());
      }
    }
  }
  else
  {
    ::std::stringstream ss;
    ss << "Failed to open " << myCastepRun.getCellFile() << " for writing.";
    return OptimisationOutcome::failure(OptimisationError::INTERNAL_ERROR, ss.str());
  }
  return OptimisationOutcome::success();
}

OptimisationOutcome CastepGeomOptRun::doRelaxation(
  common::Structure &structure,
  const common::AtomSpeciesDatabase & speciesDb,
  const fs::path & castepExe)
{
  if(myCastepRun.runCastep(castepExe) != CastepRunResult::SUCCESS)
  {
    ::std::stringstream ss;
    ss << "Failed to run castep with: " << castepExe.string() << " " << myOrigCellFile.stem();
    return OptimisationOutcome::failure(OptimisationError::INTERNAL_ERROR, ss.str());
  }

  // Read in results from castep run and update structure
  const CastepRunResult::Value updateResult = myCastepRun.updateStructureFromOutput(structure, speciesDb);
  if(updateResult != CastepRunResult::SUCCESS)
  {
    if(updateResult == CastepRunResult::OUTPUT_NOT_FOUND)
    {
      ::std::stringstream ss;
      ss << "Castep output: " << myCastepRun.getCastepFile().string() << " not found.";
      return OptimisationOutcome::failure(OptimisationError::INTERNAL_ERROR, ss.str());
    }
    else if(updateResult == CastepRunResult::FAILED_TO_READ_STRUCTURE)
    {
      ::std::stringstream ss;
      ss << "Failed to read structure from " << myCastepRun.getCastepFile().string() << ".";
      return OptimisationOutcome::failure(OptimisationError::INTERNAL_ERROR, ss.str());
    }
    else
      return OptimisationOutcome::failure(OptimisationError::INTERNAL_ERROR);
  }
  return OptimisationOutcome::success();
}

const ::std::string FINAL_ENTHALPY("Final Enthalpy"); // TODO: REMOVE

bool CastepGeomOptRun::updateOptimisationInfo(
  common::Structure & structure,
  PotentialData & data,
  const common::AtomSpeciesDatabase & speciesDb
)
{
  fs::ifstream * castepFileStream;
  myCastepRun.openCastepFile(&castepFileStream);

  bool readSuccessfully = false;
  std::string line;
  if(io::findFirstLine(line, *castepFileStream, FINAL_ENTHALPY))
  {
    double enthalpy;
    if(io::findFirstFloat(enthalpy, line))
    {
      data.internalEnergy = enthalpy;
      readSuccessfully = true;
    }
  }

  return readSuccessfully;
}

bool CastepGeomOptRun::optimistaionSucceeded()
{
  fs::ifstream * castepFileStream;
  myCastepRun.openCastepFile(&castepFileStream);

  bool succeeded = false;
  ::std::string line;
  if(io::findLastLine(line, *castepFileStream, "Geometry optimization"))
  {
    if(::boost::find_first(line, "completed successfully"))
      succeeded = true;
  }

  return succeeded;
}

}
}

