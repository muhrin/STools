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
#include "potential/OptimisationSettings.h"

// NAMESPACES ////////////////////////////////
namespace sstbx {
namespace potential {

namespace fs = ::boost::filesystem;

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
  OptimisationData optimisationData;
  return optimise(structure, optimisationData, options);
}

OptimisationOutcome CastepGeomOptimiser::optimise(
	common::Structure & structure,
  OptimisationData & optimisationData,
  const OptimisationSettings & options) const
{
  const ::std::string outSeedName(structure.getName());

  detail::CastepGeomOptRun geomOpt(
    options,
    myCastepSeed,
    outSeedName,
    myKeepIntermediates,
    myCellReaderWriter,
    myCastepReader
  );

  const OptimisationOutcome outcome =
    geomOpt.runFullRelax(structure, optimisationData, myCastepExe, mySpeciesDb);
  optimisationData.saveToStructure(structure);
  
  return outcome;
}

namespace detail {

const int CastepGeomOptRun::MAX_RELAX_ATTEMPTS = 20;

CastepGeomOptRun::CastepGeomOptRun(
  const OptimisationSettings & optimisationSettings,
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
myCastepReader(castepReader),
myOptimisationSettings(optimisationSettings)
{}

CastepGeomOptRun::~CastepGeomOptRun()
{
  if(!myKeepIntermediates)
    myCastepRun.deleteAllFiles();
}

OptimisationOutcome CastepGeomOptRun::runFullRelax(
  common::Structure & structure,
  OptimisationData & data,
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
 
  doPreRelaxation(structure, data, speciesDb, castepExe);

  OptimisationOutcome outcome;
  int successfulRelaxations = 0;
  int i;
  for(i = 0; successfulRelaxations < numRelaxations && i < MAX_RELAX_ATTEMPTS; ++i)
  {
    outcome = doRelaxation(structure, data, speciesDb, castepExe);
    if(!outcome.isSuccess())
      return outcome;

    // Keep relaxing until we get somewhere
    if(optimisationSucceeded())
      ++successfulRelaxations;
  }
  // How did we exit?
  if(i == MAX_RELAX_ATTEMPTS)
    return OptimisationOutcome::failure(OptimisationError::FAILED_TO_CONVERGE);

  return OptimisationOutcome::success();
}

OptimisationOutcome CastepGeomOptRun::updateStructure(
  common::Structure & structure,
  OptimisationData & data,
  const common::AtomSpeciesDatabase & speciesDb
)
{
  const CastepRunResult::Value updateResult = myCastepRun.updateStructureFromOutput(structure, speciesDb);
  if(updateResult != CastepRunResult::SUCCESS)
  {
    if(updateResult == CastepRunResult::OUTPUT_NOT_FOUND)
    {
      ::std::stringstream ss;
      ss << "Castep output: " << myCastepRun.getCellOutFile().string() << " not found.";
      return OptimisationOutcome::failure(OptimisationError::INTERNAL_ERROR, ss.str());
    }
    else if(updateResult == CastepRunResult::FAILED_TO_READ_STRUCTURE)
    {
      ::std::stringstream ss;
      ss << "Failed to read structure from " << myCastepRun.getCellOutFile().string() << ".";
      return OptimisationOutcome::failure(OptimisationError::INTERNAL_ERROR, ss.str());
    }
    else
      return OptimisationOutcome::failure(OptimisationError::INTERNAL_ERROR);
  }
  parseOptimisationInfo(structure, data, speciesDb);
  data.saveToStructure(structure);

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
  if(myCastepRun.openNewCellFile(&newCellFileStream) == CastepRunResult::SUCCESS)
  {  
    myCellReaderWriter.writeStructure(*newCellFileStream, structure, speciesDb);

    if(myOptimisationSettings.pressure)
    {
      *newCellFileStream << ::std::endl;
      myCastepRun.writePressure(*myOptimisationSettings.pressure);
    }

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

OptimisationOutcome CastepGeomOptRun::doPreRelaxation(
  common::Structure & structure,
  OptimisationData & optimisationData,
  const common::AtomSpeciesDatabase & speciesDb,
  const ::boost::filesystem::path & castepExe
)
{
  const fs::path origParamFile(myCastepRun.getParamFile().string() + ".orig");
  fs::copy_file(myCastepRun.getParamFile(), origParamFile, fs::copy_option::overwrite_if_exists);

  CastepRun::ParamsMap paramsMap;
  paramsMap["geom_max_iter"] = "2";
  myCastepRun.insertParams(paramsMap);

  // Do short relaxations
  for(size_t i = 0;  i < 2;  ++i)
    doRelaxation(structure, optimisationData, speciesDb, castepExe);

  // Copy the original back
  fs::copy_file(origParamFile, myCastepRun.getParamFile(), fs::copy_option::overwrite_if_exists);
  fs::remove_all(origParamFile);

  return OptimisationOutcome::success();
}

OptimisationOutcome CastepGeomOptRun::doRelaxation(
  common::Structure &structure,
  OptimisationData & optimistaionData,
  const common::AtomSpeciesDatabase & speciesDb,
  const fs::path & castepExe)
{
  // 1. Write the .cell file from the current structure
  OptimisationOutcome outcome = makeCellCopy(structure, speciesDb);
  if(!outcome.isSuccess())
    return outcome;

  // 2. Run Castep
  if(myCastepRun.runCastep(castepExe) != CastepRunResult::SUCCESS)
  {
    ::std::stringstream ss;
    ss << "Failed to run castep with: " << castepExe.string() << " " << io::stemString(myCastepRun.getParamFile());
    return OptimisationOutcome::failure(OptimisationError::INTERNAL_ERROR, ss.str());
  }

  if(!myCastepRun.finishedSuccessfully())
    return OptimisationOutcome::failure(OptimisationError::INTERNAL_ERROR, "Castep did not finish correctly.");

  // 3. Read in results from -out.cell and update structure
  return updateStructure(structure, optimistaionData, speciesDb);
}

bool CastepGeomOptRun::optimisationSucceeded()
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


bool CastepGeomOptRun::parseOptimisationInfo(
  common::Structure & structure,
  OptimisationData & data,
  const common::AtomSpeciesDatabase & speciesDb
)
{
  static const ::std::string FINAL_ENTHALPY("Final Enthalpy");
  static const ::std::string FORCES("* Forces *");
  static const ::std::string STRESS_TENSOR("* Stress Tensor *");

  static const ::boost::regex RE_TENSOR_ROW(
    ::std::string("[x|y|z][[:blank:]]+") +
    "(" + io::PATTERN_FLOAT + ")[[:blank:]]+" +
    "(" + io::PATTERN_FLOAT + ")[[:blank:]]+" +
    "(" + io::PATTERN_FLOAT + ")"
  );

  fs::ifstream * castepFileStream;
  myCastepRun.openCastepFile(&castepFileStream);

  bool readSuccessfully = true;
  std::string line;

  // Enthalpy
  if(io::findLastLine(line, *castepFileStream, FINAL_ENTHALPY))
  {
    double enthalpy;
    if(io::findFirstFloat(enthalpy, line))
    {
      data.enthalpy.reset(enthalpy);
    }
  }
  else
    readSuccessfully = false;

  // Forces
  if(io::findNextLine(line, *castepFileStream, FORCES))
  {
    // TODO: READ FORCES
  }
  else
    readSuccessfully = false;

  // Stress tensor and pressure
  if(io::findNextLine(line, *castepFileStream, STRESS_TENSOR))
  {
    ::arma::mat33 stressTensor;
    ::std::string tensorLine;
    int row = 0;
    while(::std::getline(*castepFileStream, tensorLine) &&
      tensorLine.size() > 2 &&
      tensorLine.substr(0, 2) == " *")
    {
      ::boost::smatch match;
      ::std::string x, y, z;
      if(::boost::regex_search(tensorLine, match, RE_TENSOR_ROW))
      {
        x.assign(match[1].first, match[1].second);
        y.assign(match[3].first, match[3].second);
        z.assign(match[5].first, match[5].second);
        try
        {
          stressTensor(row, 0) = ::boost::lexical_cast<double>(x);
          stressTensor(row, 1) = ::boost::lexical_cast<double>(y);
          stressTensor(row, 2) = ::boost::lexical_cast<double>(z);
          ++row;
        }
        catch(const ::boost::bad_lexical_cast & /*e*/)
        {
          readSuccessfully = false;
        }
      }
      if(row == 3)
      {
        data.stressMtx.reset(stressTensor);
        data.pressure.reset(-::arma::trace(stressTensor) / 3.0);
        break;
      }
    }
    if(row != 3)
      readSuccessfully = false;
  }
  else
    readSuccessfully = false;

  // Internal energy
  if(readSuccessfully)
    data.internalEnergy.reset(*data.enthalpy -
    *data.pressure * structure.getUnitCell()->getVolume()
  );

  return readSuccessfully;
}

} // namespace detail

}
}

