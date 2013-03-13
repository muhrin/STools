/*
 * WriteStructure.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "blocks/WriteStructure.h"

// From SSTbx
#include <common/Structure.h>
#include <io/StructureReadWriteManager.h>
#include <io/BoostFilesystem.h>
#include <utility/UtilFunctions.h>

// From local
#include "common/SharedData.h"
#include "common/StructureData.h"
#include "common/UtilityFunctions.h"


// NAMESPACES ////////////////////////////////


namespace spipe {
namespace blocks {

namespace fs = ::boost::filesystem;
namespace ssc = ::sstbx::common;
namespace ssio = ::sstbx::io;
namespace ssu = ::sstbx::utility;

const bool WriteStructure::WRITE_MULTI_DEFAULT = true;

WriteStructure::WriteStructure():
SpBlock("Write structures"),
myWriteMultiStructure(WRITE_MULTI_DEFAULT),
myState(State::DISABLED)
{}

bool WriteStructure::getWriteMulti() const
{
  return myWriteMultiStructure;
}

void WriteStructure::setWriteMulti(const bool writeMulti)
{
  myWriteMultiStructure = true;
}

const ::std::string & WriteStructure::getFileType() const
{
  return myFileType;
}

void WriteStructure::setFileType(const ::std::string & extension)
{
  myFileType = extension;
}

void WriteStructure::pipelineStarting()
{
  const ssio::StructureReadWriteManager & rwMan = getRunner()->memory().global().getStructureIo();

  // Do we want to use a custom writer and does it exist?
  if(!myFileType.empty() && rwMan.getWriter(myFileType))
    myState = State::USE_CUSTOM_WRITER;
  else if(rwMan.getDefaultWriter()) // Try the default
    myState = State::USE_DEFAULT_WRITER;
  else
  {
    // TODO: Log error
    myState = State::DISABLED;
  }
}

void WriteStructure::in(::spipe::common::StructureData & data)
{
  const ssio::StructureReadWriteManager & rwMan = getRunner()->memory().global().getStructureIo();
  if(myState != State::DISABLED)
  {
    common::SharedData & shared = getRunner()->memory().shared();
    ssc::Structure * const structure = data.getStructure();

    const ssio::IStructureWriter * writer = NULL;
    
    if(myState == State::USE_CUSTOM_WRITER)
      writer = rwMan.getWriter(myFileType);
    else
      writer = rwMan.getDefaultWriter();

    if(writer)
    {
      const ssio::ResourceLocator saveLocation(generateLocator(*structure, *writer));

      bool writeSuccessful;
      if(myState == State::USE_CUSTOM_WRITER)
        writeSuccessful = rwMan.writeStructure(*data.getStructure(), saveLocation, getRunner()->memory().global().getSpeciesDatabase(), myFileType);
      else
        writeSuccessful = rwMan.writeStructure(*data.getStructure(), saveLocation, getRunner()->memory().global().getSpeciesDatabase());
    	
      if(writeSuccessful)
      {
        // TODO: Produce error
      }
    }
    else
    {
      // TODO: Error couldn't find writer
    }
  }
	out(data);
}

ssio::ResourceLocator
WriteStructure::generateLocator(
  ssc::Structure & structure,
  const ssio::IStructureWriter & writer) const
{
  // Check if the structure has a name already, otherwise give it one
  if(structure.getName().empty())
	  structure.setName(ssu::generateUniqueName());

  // Create the path to store the structure
  fs::path p(getRunner()->memory().shared().getOutputPath(*getRunner()));

  // Should all the structures be stored in one file or seaprate files?
  if(useMultiStructure(writer))
    p /= getRunner()->memory().shared().getOutputFileStem();
  else
    p /= fs::path(structure.getName());

  // Make sure to add on the extension if we're using a custom file type
  if(myState == State::USE_CUSTOM_WRITER)
    p = p.string() + "." + myFileType;

  return ssio::ResourceLocator(p, structure.getName());
}

bool WriteStructure::useMultiStructure(const ssio::IStructureWriter & writer) const
{
  if(myWriteMultiStructure && writer.multiStructureSupport())
    return true;

  return false;
}

}
}

