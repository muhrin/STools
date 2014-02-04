/*
 * SearchStoichiometries.cpp
 *
 *  Created on: May 4, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "blocks/SearchStoichiometries.h"

#include <boost/foreach.hpp>
#include <boost/optional.hpp>
#include <boost/lexical_cast.hpp>

#include <spl/build_cell/GenerationSettings.h>
#include <spl/common/Structure.h>
#include <spl/io/BoostFilesystem.h>
#include <spl/io/ResourceLocator.h>
#include <spl/utility/MultiIdx.h>

// Local includes
#include "common/CommonData.h"
#include "common/PipeFunctions.h"
#include "common/StructureData.h"
#include "common/SharedData.h"
#include "common/UtilityFunctions.h"
#include "utility/DataTable.h"

namespace spipe {
namespace blocks {

// NAMESPACE ALIASES /////////////////////////
namespace fs = boost::filesystem;
namespace common = spipe::common;
namespace ssbc = spl::build_cell;
namespace ssc = spl::common;
namespace ssio = spl::io;
namespace ssu = spl::utility;
namespace structure_properties = ssc::structure_properties;

const size_t SearchStoichiometries::Options::UNLIMITED_ATOMS =
    std::numeric_limits< size_t>::max();

SearchStoichiometries::SearchStoichiometries(const Options & options,
    BlockHandle & subpipe) :
    Block("Search stoichiometries"), myOptions(options), mySubpipe(subpipe), mySubpipeEngine(
        NULL)
{
}

void
SearchStoichiometries::pipelineInitialising()
{
  myTableSupport.setFilename(outputFileStemPath().string() + ".stoich");
  myTableSupport.registerEngine(getEngine());
}

void
SearchStoichiometries::start()
{
  using boost::lexical_cast;
  using std::string;

  // Start looping over the possible stoichiometries
  size_t totalAtoms = 0;

  const fs::path workingDir = this->workingDir();

  const ssu::MultiIdxRange< unsigned int> & stoichRange = getStoichRange();
  BOOST_FOREACH(const ssu::MultiIdx<unsigned int> & currentIdx, stoichRange)
  {
    // Need to get the shared data each time as it may have been reset after each
    // run of the pipe
    SharedDataType & sweepPipeData = mySubpipeEngine->sharedData();

    totalAtoms = currentIdx.sum();
    if(totalAtoms == 0 || totalAtoms > myOptions.maxAtoms)
      continue;

    ssbc::GenerationSettings generationSettings;

    // Insert all the atoms
    std::stringstream stoichStringStream;
    size_t numAtomsOfSpecies;
    ssc::AtomSpeciesId::Value species;
    size_t i = 0;
    BOOST_FOREACH(AtomRanges::const_reference range, myOptions.atomRanges)
    {
      numAtomsOfSpecies = currentIdx[i];
      generationSettings.atomsSettings[range.first].num = numAtomsOfSpecies;

      stoichStringStream << numAtomsOfSpecies;

      // Append the species symbol
      if(!range.first.empty())
        stoichStringStream << range.first;

      // Add delimiter apart from for last species
      if(i + 1 < currentIdx.dims())
        stoichStringStream << "-";
      ++i;
    } // End loop over atoms

    // Set the stoichiometry in shared data
    getEngine()->globalData().objectsStore[common::GlobalKeys::GENERATION_SETTINGS] =
        generationSettings;

    // Append the species ratios to the output directory name
    fs::path sweepPath = workingDir;
    if(myOptions.useSeparateDirectories)
      sweepPath /= stoichStringStream.str();
    // Set a directory for this set of parameters
    sweepPipeData.setWorkingDir(sweepPath);

    // Start the sweep pipeline
    mySubpipeEngine->run();

    // Update the table
    updateTable(sweepPath.string(), currentIdx);

    // Send any finished structure data down my pipe, this will also
    // update the table with any information from the buffered structures
    releaseBufferedStructures(sweepPath.string());

  } // End loop over stoichiometries
}

void
SearchStoichiometries::finished(StructureDataUniquePtr data)
{
  // Register the data with our pipeline to transfer ownership
  // and save it in the buffer for sending down the pipe
  myBuffer.push_back(getEngine()->registerData(data));
}

void
SearchStoichiometries::releaseBufferedStructures(
    const utility::DataTable::Key & tableKey)
{
  // Send any finished structure data down my pipe
  utility::DataTable & table = myTableSupport.getTable();

  ssio::ResourceLocator lastSavedRelative;

  const ssc::Structure * structure;
  const double * internalEnergy;

  unsigned int * spacegroup;
  BOOST_FOREACH(StructureDataTyp * const strData, myBuffer)
  {
    structure = strData->getStructure();
    lastSavedRelative = strData->getRelativeSavePath(workingDir());

    if(!lastSavedRelative.empty())
    {
      // Insert the relative path to the last place this structre was saved
      table.insert(tableKey, "lowest", lastSavedRelative.string());
    }

    spacegroup = strData->objectsStore.find(
        structure_properties::general::SPACEGROUP_NUMBER);
    if(spacegroup)
    {
      table.insert(tableKey, "sg",
          boost::lexical_cast< std::string>(*spacegroup));
    }

    // Try to calculate the energy/atom
    internalEnergy = structure->getProperty(
        structure_properties::general::ENERGY_INTERNAL);
    if(internalEnergy)
    {
      const size_t numAtoms = structure->getNumAtoms();
      if(numAtoms != 0)
        table.insert(tableKey, "energy/atom",
            common::toString(*internalEnergy / numAtoms));
    }

    // Pass the structure on
    out(strData);
  }
  myBuffer.clear();
}

void
SearchStoichiometries::engineAttached(Engine::SetupType * const setup)
{
  mySubpipeEngine = setup->createEngine();
  mySubpipeEngine->attach(mySubpipe);

  // Set outselves to collect any finished data from the sweep pipeline
  mySubpipeEngine->setFinishedDataSink(this);
}

void
SearchStoichiometries::engineDetached()
{
  if(mySubpipeEngine)
  {
    mySubpipeEngine->setFinishedDataSink(NULL);
    mySubpipeEngine->detach();
    mySubpipeEngine = NULL;
  }
}

ssu::MultiIdxRange< unsigned int>
SearchStoichiometries::getStoichRange()
{
  const size_t numSpecies = myOptions.atomRanges.size();

  ssu::MultiIdx< unsigned int> lower(numSpecies), upper(numSpecies);

  size_t i = 0;
  BOOST_FOREACH(AtomRanges::const_reference range, myOptions.atomRanges)
  {
    lower[i] = range.second.min();
    // Need to add one as the extents is a half-open interval i.e. [x0, x1)
    upper[i] = range.second.max() + 1;
    ++i;
  }

  return ssu::MultiIdxRange< unsigned int>(lower, upper);
}

void
SearchStoichiometries::updateTable(const utility::DataTable::Key & key,
    const ssu::MultiIdx< unsigned int> & currentIdx)
{
  using boost::lexical_cast;
  using std::string;

  utility::DataTable & table = myTableSupport.getTable();

  ssc::AtomSpeciesId::Value species;
  string speciesTableColumn;
  size_t numAtomsOfSpecies;
  size_t i = 0;
  BOOST_FOREACH(AtomRanges::const_reference range, myOptions.atomRanges)
  {
    numAtomsOfSpecies = currentIdx[i];

    if(!range.first.empty())
      speciesTableColumn = range.first;
    else
      speciesTableColumn = lexical_cast< string>(i);

    table.insert(key, speciesTableColumn,
        lexical_cast< string>(numAtomsOfSpecies));
    ++i;
  } // End loop over atoms
}

}
}

