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
#include "os/Process.h"
#include "potential/PotentialData.h"

// NAMESPACES ////////////////////////////////
namespace sstbx {
namespace potential {

namespace fs = ::boost::filesystem;

const ::std::string CastepGeomOptimiser::FINAL_ENTHALPY("Final Enthalpy");

class CastepHelper
{
public:
  CastepHelper(
    const ::std::string & originalSeed,
    const ::std::string & newSeed,
    const bool keepIntermediates);
  ~CastepHelper();
  
  bool copyParamFile() const;
  const fs::path & getOrigParam() const;
  const fs::path & getNewParam() const;
  const fs::path & getOrigCell() const;
  const fs::path & getNewCell() const;
  bool openOrigCell(fs::ifstream * & ifstream);
  bool openNewCell(fs::ofstream * & ofstream);
  void closeStreams();

private:
  bool deleteIntermediateFiles() const;

  const fs::path myOrigCell;
  const fs::path myOrigParam;
  const fs::path myNewCell;
  const fs::path myNewParam;
  fs::ifstream myOrigCellStream;
  fs::ofstream myNewCellStream;
  const bool myKeepIntermediates;
};

CastepHelper::CastepHelper(
  const ::std::string & originalSeed,
  const ::std::string & newSeed,
  const bool keepIntermediates):
myOrigCell(originalSeed + ".cell"),
myOrigParam(originalSeed + ".param"),
myNewCell(newSeed + ".cell"),
myNewParam(newSeed + ".param"),
myKeepIntermediates(keepIntermediates)
{}

CastepHelper::~CastepHelper()
{
  closeStreams();
  if(!myKeepIntermediates)
    deleteIntermediateFiles();
}

bool CastepHelper::openOrigCell(fs::ifstream * & ifstream)
{
  if(!myOrigCellStream.is_open())
    myOrigCellStream.open(myOrigCell);

  ifstream = &myOrigCellStream;
  return true;
}
  
bool CastepHelper::openNewCell(fs::ofstream * & ofstream)
{
  if(!myNewCellStream.is_open())
    myNewCellStream.open(myNewCell);

  ofstream = &myNewCellStream;
  return true;
}

bool CastepHelper::copyParamFile() const
{
  // Copy over the .param file
  try
  {
    fs::copy_file(myOrigParam, myNewParam,  fs::copy_option::overwrite_if_exists);
  }
  catch(const fs::filesystem_error & /*e*/)
  {
    return false;
  }
  return true;
}

const fs::path & CastepHelper::getOrigParam() const
{
  return myOrigParam;
}

const fs::path & CastepHelper::getNewParam() const
{
  return myNewParam;
}

const fs::path & CastepHelper::getOrigCell() const
{
  return myOrigCell;
}

const fs::path & CastepHelper::getNewCell() const
{
  return myNewCell;
}

void CastepHelper::closeStreams()
{
  if(myOrigCellStream.is_open())
    myOrigCellStream.close();
  if(myNewCellStream.is_open())
    myNewCellStream.close();
}

bool CastepHelper::deleteIntermediateFiles() const
{
  fs::remove_all(myNewParam);
  fs::remove_all(myNewCell);
  //fs::remove_all(myNewParam);
  return true;
}

CastepGeomOptimiser::CastepGeomOptimiser(
  const ::boost::filesystem::path & castepExe,
  const ::std::string & castepSeedPath,
  const bool keepIntermediates
):
myCastepExe(castepExe),
myCastepSeedPath(castepSeedPath),
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

  CastepHelper helper(myCastepSeedPath, outSeedName, myKeepIntermediates);

  if(!fs::exists(helper.getOrigParam()))
  {
    ::std::stringstream ss;
    ss << "Castep input file " << helper.getOrigParam() << " not found.";
    return OptimisationOutcome::failure(OptimisationError::INTERNAL_ERROR, ss.str());
  }

  if(!helper.copyParamFile())
  {
    ::std::stringstream ss;
    ss << "Failed to copy " << helper.getOrigParam() << " to " << helper.getNewParam() << ".";
    return OptimisationOutcome::failure(OptimisationError::INTERNAL_ERROR, ss.str());
  }
  
  fs::ofstream * newCellStream;
  if(helper.openNewCell(newCellStream))
  {
    myCellReaderWriter.writeStructure(*newCellStream, structure, mySpeciesDb);

    // Now copy over the original contents
    if(fs::exists(helper.getOrigCell()))
    {
      fs::ifstream * origCellStream;
      if(helper.openOrigCell(origCellStream))
      {
        *newCellStream << ::std::endl << "#==ORIGINAL CONTENTS==" << ::std::endl;
        *newCellStream << origCellStream->rdbuf();
      }
      else
      {
        ::std::stringstream ss;
        ss << "Failed to open " << helper.getOrigCell() << " for reading.";
        return OptimisationOutcome::failure(OptimisationError::INTERNAL_ERROR, ss.str());
      }
    }
  }
  else
  {
    ::std::stringstream ss;
    ss << "Failed to open " << helper.getNewCell() << " for writing.";
    return OptimisationOutcome::failure(OptimisationError::INTERNAL_ERROR, ss.str());
  }
  helper.closeStreams();
  
  // TODO: Run castep with outSeedName as parameter
  ::std::vector< ::std::string> args(1, outSeedName);
  if(os::runBlocking(myCastepExe, args) != 0)
  {
    ::std::stringstream ss;
    ss << "Failed to run castep with: " << myCastepExe << " " << args[0];
    return OptimisationOutcome::failure(OptimisationError::INTERNAL_ERROR, ss.str());
  }

  // Read in results from castep run and update structure
  const fs::path seedCastep(outSeedName + ".castep");
  if(fs::exists(seedCastep))
  {
    updateStructure(structure, data, seedCastep, mySpeciesDb);
    if(!myKeepIntermediates)
      fs::remove_all(seedCastep);
  }

  return OptimisationOutcome::success();
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

