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

  geomOpt.runFullRelax(structure, data, myCastepExe, mySpeciesDb);

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

