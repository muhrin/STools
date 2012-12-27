/*
 * StoichiometrySearch.cpp
 *
 *  Created on: May 4, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "blocks/StoichiometrySearch.h"

#include <boost/foreach.hpp>
#include <boost/optional.hpp>
#include <boost/lexical_cast.hpp>

// From SSTbx
#include <build_cell/AtomsDescription.h>
#include <build_cell/StructureDescription.h>
#include <build_cell/Types.h>
#include <common/AtomSpeciesDatabase.h>
#include <common/Structure.h>
#include <io/BoostFilesystem.h>
#include <io/ResourceLocator.h>
#include <utility/MultiIdx.h>

// Local includes
#include "common/StructureData.h"
#include "common/SharedData.h"
#include "common/UtilityFunctions.h"
#include "utility/DataTable.h"


namespace spipe {
namespace blocks {

// NAMESPACE ALIASES /////////////////////////
namespace fs = ::boost::filesystem;
namespace common = ::spipe::common;
namespace ssbc = ::sstbx::build_cell;
namespace ssc = ::sstbx::common;
namespace ssio = ::sstbx::io;
namespace ssu = ::sstbx::utility;
namespace structure_properties = ssc::structure_properties;


StoichiometrySearch::StoichiometrySearch(
  const ::sstbx::common::AtomSpeciesId::Value  species1,
  const ::sstbx::common::AtomSpeciesId::Value  species2,
  const size_t maxAtoms,
  SpStartBlock & subpipe):
SpBlock("Sweep stoichiometry"),
myMaxAtoms(maxAtoms),
mySubpipe(subpipe)
{
  mySpeciesParameters.push_back(SpeciesParameter(species1, maxAtoms));
  mySpeciesParameters.push_back(SpeciesParameter(species2, maxAtoms));
}

StoichiometrySearch::StoichiometrySearch(
  const SpeciesParamters & speciesParameters,
  const size_t       maxAtoms,
  const double       atomsRadius,
  SpStartBlock &   sweepPipe):
SpBlock("Sweep stoichiometry"),
mySpeciesParameters(speciesParameters),
myMaxAtoms(maxAtoms),
mySubpipe(sweepPipe),
myTableSupport(fs::path("stoich.dat"))
{}

void StoichiometrySearch::pipelineInitialising()
{
  myTableSupport.setFilename(getRunner()->memory().global().getOutputFileStem().string() + ".stoich");
  myTableSupport.registerRunner(*getRunner());
}

void StoichiometrySearch::pipelineStarting()
{
  // Save where the pipeline will be writing to
	myOutputPath = getRunner()->memory().shared().getOutputPath(*getRunner());
}


void StoichiometrySearch::start()
{
  using ::boost::lexical_cast;
  using ::std::string;

  SharedDataType & sweepPipeData = mySubpipeRunner->memory().shared();
  const ssc::AtomSpeciesDatabase & atomsDb = getRunner()->memory().global().getSpeciesDatabase();

  // Start looping over the possible stoichiometries
  size_t totalAtoms = 0;
  ::std::string sweepPipeOutputPath;

  const ssu::MultiIdxRange<unsigned int> stoichRange = getStoichRange();
  BOOST_FOREACH(const ssu::MultiIdx<unsigned int> & currentIdx, stoichRange)
  {
    totalAtoms = currentIdx.sum();
    if(totalAtoms == 0 || totalAtoms > myMaxAtoms)
      continue;

    // Set the current structure description in our subpipline
    StrDescPtr structureDescription(
      new ::sstbx::build_cell::StructureDescription(ssbc::ConstUnitCellBlueprintPtr(new ssbc::RandomUnitCell()))
    );
    sweepPipeData.setStructureDescription(structureDescription);

    // Insert all the atoms
    ::std::stringstream stoichStringStream;
    size_t                    numAtomsOfSpecies;
    ssc::AtomSpeciesId::Value species;
    const string *            speciesSymbol;
    for(size_t i = 0; i < currentIdx.dims(); ++i)
    {
      species           = mySpeciesParameters[i].id;
      speciesSymbol     = atomsDb.getSymbol(species);
      numAtomsOfSpecies = currentIdx[i];
  
      if(numAtomsOfSpecies > 0)
      {
        structureDescription->addChild(ssbc::AtomsDescriptionPtr(
          new ::sstbx::build_cell::AtomsDescription(mySpeciesParameters[i].id, numAtomsOfSpecies)
        ));
      }

      stoichStringStream << numAtomsOfSpecies;


      // Append the species symbol
      if(speciesSymbol)
        stoichStringStream << *speciesSymbol;

      // Add delimiter apart from for last species
      if(i + 1 < currentIdx.dims())
        stoichStringStream << "-";

    } // End loop over atoms

    // Append the species ratios to the output directory name
    sweepPipeData.appendToOutputDirName(stoichStringStream.str());

    // Find out the pipeline relative path to where all the structures are going to be saved
    sweepPipeOutputPath = sweepPipeData.getPipeRelativeOutputPath().string();

    // Start the sweep pipeline
    mySubpipeRunner->run();

    // Update the table
    updateTable(sweepPipeOutputPath, currentIdx, atomsDb);

    // Send any finished structure data down my pipe, this will also
    // update the table with any information from the buffered structures
    releaseBufferedStructures(sweepPipeOutputPath);

  } // End loop over stoichiometries
}

void StoichiometrySearch::finished(SpStructureDataPtr data)
{
  // Register the data with our pipeline to transfer ownership
  // and save it in the buffer for sending down the pipe
  myBuffer.push_back(&getRunner()->registerData(data));
}

void StoichiometrySearch::releaseBufferedStructures(
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
    lastSavedRelative = strData->getRelativeSavePath(*getRunner());

    if(!lastSavedRelative.empty())
    {
      // Insert the relative path to the last place this structre was saved
      table.insert(tableKey, "lowest", lastSavedRelative.string());
    }

    spacegroup = strData->objectsStore.find(structure_properties::general::SPACEGROUP_NUMBER);
    if(spacegroup)
    {
      table.insert(tableKey, "sg", ::boost::lexical_cast< ::std::string>(*spacegroup));
    }

    // Try to calculate the energy/atom
    internalEnergy = structure->getProperty(structure_properties::general::ENERGY_INTERNAL);
    if(internalEnergy)
    {
      const size_t numAtoms = structure->getNumAtoms();
      if(numAtoms != 0)
        table.insert(tableKey, "energy/atom", common::getString(*internalEnergy / numAtoms));
    }
    

    // Pass the structure on
		out(*strData);
	}
	myBuffer.clear();
}

void StoichiometrySearch::runnerAttached(RunnerSetupType & setup)
{
  mySubpipeRunner = setup.createChildRunner(mySubpipe);
  // Set outselves to collect any finished data from the sweep pipeline
  mySubpipeRunner->setFinishedDataSink(this);
}

ssu::MultiIdxRange<unsigned int> StoichiometrySearch::getStoichRange()
{
  const size_t numSpecies = mySpeciesParameters.size();

  ssu::MultiIdx<unsigned int> maxSpecies(numSpecies);

  for(size_t i = 0; i < numSpecies; ++i)
  {
    // Need to add one as the extents is a half-open interval i.e. [x0, x1)
    maxSpecies[i] = mySpeciesParameters[i].maxNum + 1;
  }

  return ssu::MultiIdxRange<unsigned int>(ssu::MultiIdx<unsigned int>(numSpecies), maxSpecies);
}

void StoichiometrySearch::updateTable(
  const utility::DataTable::Key &             key,
  const ssu::MultiIdx<unsigned int> & currentIdx,
  const ssc::AtomSpeciesDatabase & atomsDb)
{
  using ::boost::lexical_cast;
  using ::std::string;
  
  utility::DataTable & table = myTableSupport.getTable();

  ssc::AtomSpeciesId::Value species;
  const string *            speciesSymbol;
  string                    speciesTableColumn;
  size_t                    numAtomsOfSpecies;
  for(size_t i = 0; i < currentIdx.dims(); ++i)
  {
    species           = mySpeciesParameters[i].id;
    speciesSymbol     = atomsDb.getSymbol(species);
    numAtomsOfSpecies = currentIdx[i];

    if(speciesSymbol)
      speciesTableColumn = *speciesSymbol;
    else
      speciesTableColumn = lexical_cast<string>(i);

    table.insert(
      key,
      speciesTableColumn,
      lexical_cast<string>(numAtomsOfSpecies));

  } // End loop over atoms
}

}
}

