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

#include <pipelib/IPipeline.h>

// From SSTbx
#include <build_cell/AtomsDescription.h>
#include <build_cell/StructureDescription.h>
#include <build_cell/Types.h>
#include <common/Structure.h>
#include <utility/BoostFilesystem.h>
#include <utility/MultiIdx.h>

// Local includes
#include "common/StructureData.h"
#include "common/SharedData.h"
#include "utility/DataTable.h"

// NAMESPACES ////////////////////////////////


namespace spipe {
namespace blocks {

// NAMESPACE ALIASES /////////////////////////
namespace fs = ::boost::filesystem;
namespace common = ::spipe::common;
namespace ssbc = ::sstbx::build_cell;
namespace ssc = ::sstbx::common;
namespace ssu = ::sstbx::utility;


StoichiometrySearch::StoichiometrySearch(
  const ::sstbx::common::AtomSpeciesId::Value  species1,
  const ::sstbx::common::AtomSpeciesId::Value  species2,
  const size_t maxAtoms,
  SpPipelineTyp &				subpipe):
pipelib::Block< ::spipe::StructureDataTyp, ::spipe::SharedDataTyp>("Sweep stoichiometry"),
myMaxAtoms(maxAtoms),
mySubpipe(subpipe),
myAtomsDb(::sstbx::common::AtomSpeciesDatabase::inst())
{
  mySpeciesParameters.push_back(SpeciesParameter(species1, maxAtoms));
  mySpeciesParameters.push_back(SpeciesParameter(species2, maxAtoms));

  init();
}

StoichiometrySearch::StoichiometrySearch(
    const SpeciesParamters & speciesParameters,
    const size_t       maxAtoms,
    const double       atomsRadius,
    SpPipelineTyp &    sweepPipe):
pipelib::Block< ::spipe::StructureDataTyp, ::spipe::SharedDataTyp>("Sweep stoichiometry"),
mySpeciesParameters(speciesParameters),
myMaxAtoms(maxAtoms),
mySubpipe(sweepPipe),
myAtomsDb(::sstbx::common::AtomSpeciesDatabase::inst()),
myTableSupport(fs::path("stoich.dat"))
{
  init();
}

void StoichiometrySearch::pipelineInitialising()
{
	// Set outselves to collect any finished data from the sweep pipeline
  mySubpipe.setFinishedDataSink(*this);

  myTableSupport.setFilename(myPipeline->getGlobalData().getOutputFileStem().string() + ".stoich");
  myTableSupport.registerPipeline(*myPipeline);
}

void StoichiometrySearch::pipelineStarting()
{
  // Save where the pipeline will be writing to
	myOutputPath = myPipeline->getSharedData().getOutputPath();
}


void StoichiometrySearch::start()
{
  using ::boost::lexical_cast;
  using ::std::string;

  SharedDataTyp & sweepPipeData = mySubpipe.getSharedData();

  // Start looping over the possible stoichiometries
  size_t totalAtoms = 0;
  ::std::string sweepPipeOutputPath;

  ssu::MultiIdxRange<unsigned int> stoichRange = getStoichRange();
  BOOST_FOREACH(const ssu::MultiIdx<unsigned int> & currentIdx, stoichRange)
  {
    totalAtoms = currentIdx.sum();
    if(totalAtoms == 0 || totalAtoms > myMaxAtoms)
      continue;

    // Set the current structure description in our subpipline
    ::boost::shared_ptr<ssbc::RandomUnitCell> randomCell(new ssbc::RandomUnitCell());
    randomCell->setMaxLengthRatio(2.0);
    sweepPipeData.structureDescription = StrDescPtr(
      new ::sstbx::build_cell::StructureDescription(randomCell)
    );

    // Insert all the atoms
    ::std::stringstream stoichStringStream;
    size_t                    numAtomsOfSpecies;
    ssc::AtomSpeciesId::Value species;
    const string *            speciesSymbol;
    for(size_t i = 0; i < currentIdx.dims(); ++i)
    {
      species           = mySpeciesParameters[i].id;
      speciesSymbol     = myAtomsDb.getSymbol(species);
      numAtomsOfSpecies = currentIdx[i];
  
      if(numAtomsOfSpecies > 0)
      {
        sweepPipeData.structureDescription->addChild(
          new ::sstbx::build_cell::AtomsDescription(mySpeciesParameters[i].id, numAtomsOfSpecies)
        );
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
    mySubpipe.start();

    // Update the table
    updateTable(sweepPipeOutputPath, currentIdx);

    // Send any finished structure data down my pipe, this will also
    // update the table with any information from the buffered structures
    releaseBufferedStructures(sweepPipeOutputPath);

  } // End loop over stoichiometries
}

void StoichiometrySearch::in(StructureDataTyp * const data)
{
	SP_ASSERT(data);

	// Register the data with our pipeline to transfer ownership
	myPipeline->registerNewData(data);

	// Save it in the buffer for sending down the pipe
	myBuffer.push_back(data);
}

void StoichiometrySearch::releaseBufferedStructures(
  const utility::DataTable::Key & tableKey)
{
	// Send any finished structure data down my pipe
  utility::DataTable & table = myTableSupport.getTable();

  fs::path lastSavedRelative;

  unsigned int * spacegroup;
	BOOST_FOREACH(StructureDataTyp * const strData, myBuffer)
	{
    lastSavedRelative = strData->getRelativeSavePath(*myPipeline);

    if(!lastSavedRelative.empty())
    {
      // Insert the relative path to the last place this structre was saved
      table.insert(tableKey, "lowest", lastSavedRelative.string());
    }

    spacegroup = strData->objectsStore.find(common::StructureObjectKeys::SPACEGROUP_NUMBER);
    if(spacegroup)
    {
      table.insert(tableKey, "sg", ::boost::lexical_cast< ::std::string>(*spacegroup));
    }

    // Try to calculate the energy/atom
    if(strData->enthalpy && strData->getStructure())
    {
      const size_t numAtoms = strData->getStructure()->getNumAtoms();
      if(numAtoms != 0)
      {
        const double energyPerAtom = *strData->enthalpy / numAtoms;
        table.insert(tableKey, "energy/atom", ::boost::lexical_cast< ::std::string>(energyPerAtom));
      }
    }
    

    // Pass the structure on
		myOutput->in(*strData);
	}
	myBuffer.clear();
}

void StoichiometrySearch::init()
{
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
  const ::sstbx::utility::MultiIdx<unsigned int> & currentIdx)
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
    speciesSymbol     = myAtomsDb.getSymbol(species);
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

