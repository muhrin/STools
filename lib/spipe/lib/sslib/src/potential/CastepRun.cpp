/*
 * CastepGeomOptRun.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "potential/CastepRun.h"

#include <sstream>

#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>

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
myCellOutFile(seed + "-out.cell"),
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

const fs::path & CastepRun::getCellOutFile() const
{
  return myCellOutFile;
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

CastepRunResult::Value CastepRun::openCellOutFile(::boost::filesystem::ifstream * * ifstream)
{
  if(!fs::exists(myCellOutFile))
    return CastepRunResult::OUTPUT_NOT_FOUND;

  if(!myCellOutFileStream.is_open())
    myCellOutFileStream.open(myCellOutFile);
  else
  { // Move the file to the beginning
    myCellOutFileStream.clear(); // Clear the EoF flag
    myCellOutFileStream.seekg(0, myCellOutFileStream.beg);
  }

  if(ifstream)
    *ifstream = &myCellOutFileStream;

  return CastepRunResult::SUCCESS;
}

CastepRunResult::Value CastepRun::insertParams(ParamsMap params) const
{
  if(!fs::exists(getParamFile()))
    return CastepRunResult::INPUT_NOT_FOUND;

  const fs::path tempParamFile(getParamFile().string() + ".tmp");

  fs::ifstream paramsIn(getParamFile());
  fs::ofstream paramsOut(tempParamFile);

  ::std::string line, preComment, outLine;
  while(::std::getline(paramsIn, line))
  {
    outLine = line;

    // Get the string up the a comment character
    preComment = line.substr(0, line.find('#'));
    if(!preComment.empty() && !params.empty())
    {
      // Look through each of the parameters to insert to see if it needs to be replaced
      ParamsMap::iterator it = params.begin();
      for(const ParamsMap::const_iterator end = params.end(); it != end; ++it)
      {
        if(::boost::ifind_first(preComment, it->first))
        {
          outLine = it->first + " : " + it->second;
          break;
        }
      }
      // If we found the parameter, erase it
      if(it != params.end())
        params.erase(it);
    }
    paramsOut << outLine << ::std::endl;
  }
  paramsIn.close();
  BOOST_FOREACH(ParamsMap::const_reference entry, params)
  {
    paramsOut << entry.first << " : " << entry.second << ::std::endl;
  }
  paramsOut.close();

  // Overwrite the old with the new
  fs::remove_all(getParamFile());
  fs::copy_file(tempParamFile, getParamFile(), fs::copy_option::overwrite_if_exists);
  fs::remove_all(tempParamFile);

  return CastepRunResult::SUCCESS;
}

void CastepRun::closeAllStreams()
{
  if(myCastepFileStream.is_open())
    myCastepFileStream.close();
  if(myCellFileStream.is_open())
    myCellFileStream.close();
  if(myCellOutFileStream.is_open())
    myCellOutFileStream.close();
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
  if(!fs::exists(myCellOutFile))
    return CastepRunResult::OUTPUT_NOT_FOUND;

  CastepRunResult::Value result = openCellOutFile();
  if(result != CastepRunResult::SUCCESS)
    return result;

  common::types::StructurePtr newStructure = myCellReaderWriter.readStructure(myCellOutFileStream, speciesDb);
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
  const ::std::string stem(io::stemString(myCellFile));

  fs::remove_all(myCastepFile);
  fs::remove_all(myCellOutFile);
  fs::remove_all(stem + ".bands");
  fs::remove_all(stem + ".bib");
  fs::remove_all(stem + ".castep_bin");
  fs::remove_all(stem + ".cst_esp");
  fs::remove_all(stem + ".geom");
  fs::remove_all(stem + ".param.orig");
  fs::remove_all(stem + ".param.tmp");
  fs::remove_all(stem + ".001.err");

  return CastepRunResult::SUCCESS;
}

CastepRunResult::Value CastepRun::deleteAllFiles()
{
  deleteAllOutput();
  fs::remove_all(myCellFile);
  fs::remove_all(myParamFile);

  return CastepRunResult::SUCCESS;
}

bool CastepRun::finishedSuccessfully()
{
  CastepRunResult::Value result = openCastepFile();
  if(result != CastepRunResult::SUCCESS)
    return false;

  const ::std::string lastLine = io::getLastNonEmptyLine(myCastepFileStream);

  return ::boost::ifind_first(lastLine, "Peak") || ::boost::ifind_first(lastLine, "Total");
}

}
}

