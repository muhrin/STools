/*
 * CastepGeomOptimiser.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "potential/CastepGeomOptimiser.h"

#include <fstream>
#include <sstream>

#include <boost/algorithm/string/find.hpp>
#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>

#include "io/Parsing.h"
#include "io/BoostFilesystem.h"
#include "os/Process.h"
#include "potential/CastepRun.h"
#include "potential/PotentialData.h"

// NAMESPACES ////////////////////////////////
namespace sstbx {
namespace potential {

namespace fs = ::boost::filesystem;

const ::std::string CastepGeomOptimiser::FINAL_ENTHALPY("Final Enthalpy");


class CastepGeomOptRun
{
public:
  CastepGeomOptRun(
    const ::std::string & originalSeed,
    const ::std::string & newSeed,
    const bool keepIntermediates,
    const io::CellReaderWriter & myCellReaderWriter,
    const io::CastepReader & myCastepReader
  );
  ~CastepGeomOptRun();

  OptimisationOutcome runFullRelax(
    common::Structure & structure,
    PotentialData & data,
    const ::boost::filesystem::path & castepExe,
    const common::AtomSpeciesDatabase & speciesDb,
    const int numConsistentRelaxations = 3
  );
  bool copyParamFile() const;

private:

  static const int MAX_RELAX_ATTEMPTS;

  OptimisationOutcome makeCellCopy(
    common::Structure & structure,
    const common::AtomSpeciesDatabase & speciesDb
  );
  OptimisationOutcome doRelaxation(
    common::Structure & structure,
    const common::AtomSpeciesDatabase & speciesDb,
    const ::boost::filesystem::path & castepExe
  );
  bool updateOptimisationInfo(
    common::Structure & structure,
    PotentialData & data,
    const common::AtomSpeciesDatabase & speciesDb
  );
  bool optimistaionSucceeded();

  CastepRun myCastepRun;
  const ::boost::filesystem::path myOrigCellFile;
  const ::boost::filesystem::path myOrigParamFile;
  const bool myKeepIntermediates;
  const io::CellReaderWriter & myCellReaderWriter;
  const io::CastepReader & myCastepReader;
};


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
 
  
  OptimisationOutcome outcome;
  int successfulRelaxations = 0;
  int i;
  for(i = 0; successfulRelaxations < numRelaxations && i < MAX_RELAX_ATTEMPTS; ++i)
  {
    outcome = makeCellCopy(structure, speciesDb);
    if(!outcome.isSuccess())
      return outcome;

    outcome = doRelaxation(structure, speciesDb, castepExe);
    if(!outcome.isSuccess())
      return outcome;

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
    ss << "Failed to run castep with: " << castepExe.string() << " " << io::stemString(myCastepRun.getParamFile());
    return OptimisationOutcome::failure(OptimisationError::INTERNAL_ERROR, ss.str());
  }

  // Read in results from castep run and update structure
  const CastepRunResult::Value updateResult = myCastepRun.updateStructureFromOutput(structure, speciesDb);
  if(updateResult != CastepRunResult::SUCCESS)
  {
    if(updateResult == CastepRunResult::OUTPUT_NOT_FOUND)
    {
      ::std::stringstream ss;
      ss << "Castep output: " << myCastepRun.getCastepOutFile().string() << " not found.";
      return OptimisationOutcome::failure(OptimisationError::INTERNAL_ERROR, ss.str());
    }
    else if(updateResult == CastepRunResult::FAILED_TO_READ_STRUCTURE)
    {
      ::std::stringstream ss;
      ss << "Failed to read structure from " << myCastepRun.getCastepOutFile().string() << ".";
      return OptimisationOutcome::failure(OptimisationError::INTERNAL_ERROR, ss.str());
    }
    else
      return OptimisationOutcome::failure(OptimisationError::INTERNAL_ERROR);
  }
  return OptimisationOutcome::success();
}

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
  if(io::findFirstLine(line, *castepFileStream, CastepGeomOptimiser::FINAL_ENTHALPY))
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


CastepGeomOptimiser::CastepGeomOptimiser(
  const ::boost::filesystem::path & castepExe,
  const ::std::string & castepSeed,
  const bool keepIntermediates
):
myCastepExe(castepExe),
myCastepSeed(castepSeed),
myKeepIntermediates(keepIntermediates),
myCellReaderWriter(),
myCastepReader()
{}

OptimisationOutcome CastepGeomOptimiser::optimise(
  common::Structure & structure,
  const OptimisationSettings & options) const
{
  PotentialData potData(structure);
  return optimise(structure, potData, options);
}

OptimisationOutcome CastepGeomOptimiser::optimise(
	common::Structure & structure,
  PotentialData & data,
  const OptimisationSettings & options) const
{
  const ::std::string outSeedName(structure.getName());

  CastepGeomOptRun geomOpt(
    myCastepSeed,
    outSeedName,
    myKeepIntermediates,
    myCellReaderWriter,
    myCastepReader
  );

  return geomOpt.runFullRelax(structure, data, myCastepExe, mySpeciesDb);
}

bool CastepGeomOptimiser::updateStructure(
  common::Structure & structure,
  PotentialData & data,
  const ::boost::filesystem::path & castepFile,
  const common::AtomSpeciesDatabase & speciesDb
) const
{
  fs::ifstream castepStream(castepFile);

  common::types::StructurePtr newStructure = myCastepReader.readStructure(castepStream, speciesDb);
  if(!newStructure.get())
  {
    castepStream.close();
    return false; // Couldn't read the structure
  }

  structure.clearAtoms();
  // Copy over the unit cell
  structure.setUnitCell(makeUniquePtr(new common::UnitCell(*newStructure->getUnitCell())));
  // Copy over the atoms
  for(size_t i = 0; i < newStructure->getNumAtoms(); ++i)
    structure.newAtom(newStructure->getAtom(i));

  // Go back to the beginning of the file
  castepStream.clear(); // Clear the EoF flag
  castepStream.seekg(0, ::std::ios::beg);

  parseOptimisationData(data, castepStream);

  if(castepStream.is_open())
    castepStream.close();

  return true;
}

bool CastepGeomOptimiser::parseOptimisationData(
  PotentialData & data,
  ::std::istream & inputStream
) const
{
  std::string line;
  if(io::findFirstLine(line, inputStream, FINAL_ENTHALPY))
  {
    double enthalpy;
    if(io::findFirstFloat(enthalpy, line))
    {
      data.internalEnergy = enthalpy;
    }
  }
  return false;
}

}
}

