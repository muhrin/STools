/*
 * LoadPotStructures.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "blocks/LoadPotStructures.h"

#include <limits>
#include <vector>

#include <boost/filesystem/fstream.hpp>
#include <boost/foreach.hpp>
#include <boost/tokenizer.hpp>

// From SSTbx
#include <SSLib.h>
#include <common/Structure.h>
#include <common/Types.h>
#include <io/ResReaderWriter.h>
#include <potential/IParameterisable.h>

// Local includes
#include "PipeLibTypes.h"
#include "common/StructureData.h"
#include "common/UtilityFunctions.h"
#include "common/SharedData.h"

// NAMESPACES ////////////////////////////////


namespace spipe {
namespace blocks {

namespace ssc = ::sstbx::common;
namespace ssio = ::sstbx::io;
namespace structure_properties = ssc::structure_properties;

LoadPotStructures::LoadPotStructures(
  const sstbx::potential::IParameterisable &        pot,
  const boost::filesystem::path & potPath,
  const bool                      lowestEOnly):
SpBlock("Load potential structures"),
myPot(pot),
myPotPath(potPath),
myLowestEOnly(lowestEOnly)
{}

void LoadPotStructures::start()
{
  using boost::filesystem::ifstream;
  using boost::filesystem::path;
  using boost::tokenizer;
  using spipe::common::GlobalKeys;
  using std::getline;
  using std::string;

	typedef boost::tokenizer<boost::char_separator<char> > Tok;
	const boost::char_separator<char> sep(" \t");

  ifstream potFile(myPotPath);

  // Storage for loaded structures 
  std::vector<spipe::StructureDataTyp *> structures;

  if(potFile.is_open())
  {
	  // Load structures
    string line;

    size_t totalLoaded = 0; // The number of structures loaded in total
    size_t nLoaded = 0; // The number of structures loaded each time

    for(getline(potFile, line); potFile.good(); getline(potFile, line))
    {
      Tok tok(line, sep);
      Tok::iterator it = tok.begin();
      if(it != tok.end())
      {
        // Find the folder that this is referring to
        path strFolder(*it);

        // Load the structure(s) we're after
        nLoaded = loadStructures(strFolder, structures);
        if(nLoaded > 0)
        {
          std::pair<arma::vec, bool> result = myPot.getParamsFromString(line.substr(it->length()));
          // Was it possible to get the parameter string?
          if(result.second)
          {
            for(size_t i = 0; i < nLoaded; ++i)
            {
              StructureDataTyp * const strDat = structures[i];

              strDat->objectsStore[GlobalKeys::POTENTIAL_PARAMS] = result.first;

              // Send the structure down the pipe
              getRunner()->registerData(SpStructureDataPtr(strDat));
              out(*strDat);
            }
          } // end if(result.second)
        } // end if(nLoaded > 0)
        totalLoaded += nLoaded;
        structures.clear();
      } // end if token != end
    }
  } // end if opened pot file
}

size_t LoadPotStructures::loadStructures(
    const boost::filesystem::path &             strFolder,
    std::vector<spipe::StructureDataTyp *>  &  structureVec) const
{
  using namespace boost::filesystem;

  if(!is_directory(strFolder))
    return 0;

  sstbx::io::ResReaderWriter resReader;

  // Keep track of the lowest energy
  StructureDataTyp * lowest = NULL;
  size_t numLoaded = 0;
  ssc::Structure * structure;
  double lowestInternalEnergy = ::std::numeric_limits<double>::max();
  double * internalEnergy;

  ssio::StructuresContainer loadedStructures;

  const directory_iterator end = directory_iterator();
  for(directory_iterator dirIt = directory_iterator(strFolder);
    dirIt != end; ++dirIt)
  {
    // Get all .res files
    const path & dirEntry = dirIt->path();
    if(dirEntry.extension() == ".res")
    {
      StructureDataTyp * const strDat = new StructureDataTyp();

      // Try loading
      if(resReader.readStructures(loadedStructures, dirEntry, getRunner()->memory().global().getSpeciesDatabase()) == 1)
          continue;
      structure = &strDat->setStructure(loadedStructures.pop_back());

      internalEnergy = structure->getProperty(structure_properties::general::ENERGY_INTERNAL);

      if(myLowestEOnly)
      {
        if(lowest)
        {
          if(internalEnergy)
          {
            if(lowestInternalEnergy > *internalEnergy)
            {
              delete lowest;
              lowest = strDat;
              lowestInternalEnergy = *internalEnergy;
            }
          }
        }
        else
        {
          lowest = strDat;
          numLoaded = 1;
        }
      }
      else
        structureVec.push_back(strDat);
    }
  }

  if(myLowestEOnly)
  {
    structureVec.push_back(lowest);
  }


  return numLoaded;
}

}}

