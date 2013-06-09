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

CastepRunResult::Value CastepRun::openNewCellFile(fs::ofstream * * ofstream)
{
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

CastepRunResult::Value CastepRun::writePressure(const ::arma::mat33 & pressureTensor)
{
  if(!fs::exists(myCellFile))
    return CastepRunResult::INPUT_NOT_FOUND;

  if(!myCellFileStream.is_open())
    myCellFileStream.open(myCellFile, std::ios::out | std::ios::app);

  myCellFileStream << ::std::endl << "%BLOCK EXTERNAL_PRESSURE" << ::std::endl;

  for(unsigned int row = 0; row < 3; ++row)
  {
    myCellFileStream << ::std::string(row, ' ');
    for(unsigned int col = row; col < 3; ++col)
    {
      myCellFileStream << pressureTensor(row, col) << " ";
    }
    myCellFileStream << ::std::endl;
  }
  myCellFileStream << "%ENDBLOCK EXTERNAL_PRESSURE" << ::std::endl;

  return CastepRunResult::SUCCESS;
}

void CastepRun::closeAllStreams()
{
  if(myCastepFileStream.is_open())
    myCastepFileStream.close();
  if(myCellFileStream.is_open())
    myCellFileStream.close();
}

CastepRunResult::Value CastepRun::runCastep(const ::std::string & castepExeString)
{
  // Make sure to close all streams so we don't end up in a conflict
  closeAllStreams();

  if(!fs::exists(myCellFile) || !fs::exists(myParamFile))
    return CastepRunResult::INPUT_NOT_FOUND;

  ::std::vector< ::std::string> castepExeAndArgs;
  os::parseParameters(castepExeAndArgs, castepExeString);
  castepExeAndArgs.push_back(io::stemString(myCellFile));

  if(os::runBlocking(castepExeAndArgs) != 0)
    return CastepRunResult::FAILED_TO_RUN;

  return CastepRunResult::SUCCESS;
}

CastepRunResult::Value CastepRun::updateStructureFromOutput(
  common::Structure & structure,
  const common::AtomSpeciesDatabase & speciesDb
)
{
  CastepRunResult::Value result = openCastepFile();
  if(result != CastepRunResult::SUCCESS)
    return result;

  common::types::StructurePtr newStructure = myCastepReader.readStructure(myCastepFileStream, "last");
  if(!newStructure.get())
    return CastepRunResult::FAILED_TO_READ_STRUCTURE;

  // Copy over the structure
  structure.updateWith(*newStructure);

  return CastepRunResult::SUCCESS;
}

CastepRunResult::Value CastepRun::deleteAllOutput()
{
  closeAllStreams();
  const ::std::string stem(io::stemString(myCellFile));

  fs::remove_all(myCastepFile);
  fs::remove_all(stem + "-out.cell");
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
  static const ::std::string CASTEP_PEAK_MEMORY("Peak memory");
  static const ::std::string CASTEP_TOTAL_TIME("Total time");
  static const ::std::string CASTEP_START_LINE("|      CCC   AA    SSS  TTTTT  EEEEE  PPPP        |");

  CastepRunResult::Value result = openCastepFile();
  if(result != CastepRunResult::SUCCESS)
    return false;

  int foundPos = -1;
  ::std::string line;
  while(::std::getline(myCastepFileStream, line))
  {
    if(::boost::find_first(line, CASTEP_TOTAL_TIME) ||
      ::boost::find_first(line, CASTEP_PEAK_MEMORY))
    {
      foundPos = myCastepFileStream.tellg();
    }
  }

  // Did we find the line we were looking for?
  if(foundPos != -1)
  { // Move the stream back to that position
    myCastepFileStream.clear(); // Clear the EoF flag
    myCastepFileStream.seekg(foundPos, myCastepFileStream.beg);
  }
  else
    return false;

  // Now check that another castep run hasn't been initiated since
  if(io::findNextLine(line, myCastepFileStream, CASTEP_START_LINE))
    return false;

  return true;
}

}
}

