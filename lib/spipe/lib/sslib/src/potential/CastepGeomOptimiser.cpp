/*
 * CastepGeomOptimiser.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "potential/CastepGeomOptimiser.h"

#include <fstream>

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

bool CastepGeomOptimiser::optimise(
  common::Structure & structure,
  const OptimisationSettings & options) const
{
  PotentialData potData(structure);
  return optimise(structure, potData, options);
}

bool CastepGeomOptimiser::optimise(
	common::Structure & structure,
  PotentialData & data,
  const OptimisationSettings & options) const
{
  const ::std::string outSeedName(structure.getName());

  const fs::path origSeedParam(myCastepSeedPath + ".param");
  const fs::path origSeedCell(myCastepSeedPath + ".cell");

  if(!fs::exists(origSeedParam))
    return false; // Have to have .param file to proceed

  const fs::path seedCell(outSeedName + ".cell");
  const fs::path seedParam(outSeedName + ".param");

  // Copy over the .param file
  try
  {
    fs::copy_file(origSeedParam, seedParam,  fs::copy_option::overwrite_if_exists);
  }
  catch(const fs::filesystem_error & /*e*/)
  {
    // Couldn't copy over seed file.  Return
    return false;
  }

  fs::ofstream cellStream(seedCell);
  if(cellStream.is_open())
  {
    myCellReaderWriter.writeStructure(cellStream, structure, mySpeciesDb);

    // Now copy over the original contents
    if(fs::exists(origSeedCell))
    {
      fs::ifstream origCellStream(origSeedCell);
      if(origCellStream.is_open())
      {
        cellStream << ::std::endl << "#==ORIGINAL CONTENTS==" << ::std::endl;
        cellStream << origCellStream.rdbuf();
        cellStream.close();
      }
    }
    cellStream.close();
  }
  
  // TODO: Run castep with outSeedName as parameter
  ::std::vector< ::std::string> args(1, outSeedName);
  if(os::runBlocking(myCastepExe, args) != 0)
  {
    if(!myKeepIntermediates)
    {
      fs::remove_all(seedCell);
      fs::remove_all(seedParam);
    }
    return false;
  }

  // Read in results from castep run and update structure
  const fs::path seedCastep(outSeedName + ".castep");
  if(fs::exists(seedCastep))
  {
    updateStructure(structure, data, seedCastep, mySpeciesDb);
    if(!myKeepIntermediates)
      fs::remove_all(seedCastep);
  }

  if(!myKeepIntermediates)
  {
    fs::remove_all(seedCell);
    fs::remove_all(seedParam);
  }

  return true;
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

