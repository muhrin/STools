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
#include <build_cell/Minsep.h> // TODO: TEMPORARY
#include <build_cell/RandomCellDescription.h>
#include <build_cell/StructureDescription.h>
#include <common/Structure.h>
#include <utility/BoostFilesystem.h>
#include <utility/Loops.h>

// Local includes
#include "common/DataTableWriter.h"
#include "common/StructureData.h"
#include "common/SharedData.h"

// NAMESPACES ////////////////////////////////


namespace spipe {
namespace blocks {

namespace common = ::spipe::common;
namespace ssc = ::sstbx::common;
namespace ssu = ::sstbx::utility;

StoichiometrySearch::StoichiometrySearch(
  const ::sstbx::common::AtomSpeciesId::Value  species1,
  const ::sstbx::common::AtomSpeciesId::Value  species2,
  const size_t maxAtoms,
  const double atomsRadius,
  SpPipelineTyp &				subpipe):
pipelib::Block< ::spipe::StructureDataTyp, ::spipe::SharedDataTyp>("Convex hull"),
myMaxAtoms(maxAtoms),
myAtomsRadius(atomsRadius),
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
myAtomsRadius(atomsRadius),
mySubpipe(sweepPipe),
myAtomsDb(::sstbx::common::AtomSpeciesDatabase::inst())
{
  init();
}

StoichiometrySearch::~StoichiometrySearch()
{}

void StoichiometrySearch::pipelineInitialising()
{
	// Set outselves to collect any finished data from the sweep pipeline
  mySubpipe.setFinishedDataSink(*this);
}

void StoichiometrySearch::pipelineStarting()
{
  SP_ASSERT(myPipeline);

  using ::boost::filesystem::path;

  // Now that the pipeline is starting the pipeline output path cannot be changed so
  // get the stoichiometry output file now.
  const path outputFile = myPipeline->getSharedData().getOutputPath() / path("stoich.dat");

  // Create the data table writer
  myTableWriter.reset(new ::spipe::common::DataTableWriter(myTable, outputFile));
}

void StoichiometrySearch::pipelineFinishing()
{
  // Reset ourselves
  if(myTableWriter.get())
    myTableWriter->write();

  myTable.clear();
}

void StoichiometrySearch::start()
{
  namespace common = ::spipe::common;
  namespace ssc = ::sstbx::common;
  namespace ssu = ::sstbx::utility;
  using ::boost::lexical_cast;
  using ::boost::filesystem::path;
  using ::spipe::common::StructureObjectKeys;
  using ::std::string;

  SharedDataTyp & sweepPipeData = mySubpipe.getSharedData();

  // Start looping over the possible stoichiometries
  size_t totalAtoms = 0;
  string stoichString;
  path sweepPipeOutputPath;
  for(ssu::Loops<size_t> loops(myStoichExtents); !loops.isAtEnd(); ++loops)
  {
    const ssu::MultiIdx<size_t> & currentIdx = *loops;

    totalAtoms = currentIdx.sum();
    if(totalAtoms == 0 || totalAtoms > myMaxAtoms)
      continue;

    // Set the current structure description in our subpipline
    sweepPipeData.structureDescription = StrDescPtr(new ::sstbx::build_cell::StructureDescription());

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

    stoichString = stoichStringStream.str();

    // Update the table
    updateTable(stoichString, currentIdx);

    // Append the species ratios so the output directory name
    sweepPipeData.appendToOutputDirName(stoichStringStream.str());

    // Add a minsep constraint
    sweepPipeData.structureDescription->addAtomConstraint(new ::sstbx::build_cell::Minsep(1.5));

    // Generate the unit cell
    sweepPipeData.cellDescription = CellDescPtr(new ::sstbx::build_cell::RandomCellDescription<double>());
    sweepPipeData.cellDescription->myVolume.reset(8.0 * totalAtoms * 1.333 * 3.1415 * myAtomsRadius * myAtomsRadius * myAtomsRadius);

    // Start the sweep pipeline
    mySubpipe.start();

    // TODO : Improve this
    // Get the full path to the output directory for that subpipe run
    sweepPipeOutputPath = sweepPipeData.getOutputPath();

		// Send any finished structure data down my pipe
    const path * lastSaved;
    path lastSavedRelative;
		BOOST_FOREACH(StructureDataTyp * const sweepPipeData, myBuffer)
		{
      lastSaved = sweepPipeData->objectsStore.find(StructureObjectKeys::LAST_ABS_SAVE_PATH);

      if(lastSaved)
      {
        lastSavedRelative = *lastSaved;
        if(ssu::isAbsolute(lastSavedRelative))
        {
          lastSavedRelative = ::boost::filesystem::make_relative(sweepPipeOutputPath, lastSavedRelative);
        }
        // Insert the relative path to the last place this structre was saved
        myTable.insert(stoichString, "lowest", lastSavedRelative.string());
      }

      // Pass the structure on
			myOutput->in(*sweepPipeData);
		}
		myBuffer.clear();
  } // End loop over stoichiometries
}

void StoichiometrySearch::in(StructureDataTyp * const data)
{
	SP_ASSERT(data);

	// Register the data with our pipeline to transfer ownership
	myPipeline->registerNewData(data);

  saveTableData(*data);

	// Save it in the buffer for sending down the pipe
	myBuffer.push_back(data);
}

void StoichiometrySearch::saveTableData(const StructureDataTyp & strData)
{
  using ::spipe::common::GlobalKeys;
  using ::sstbx::common::Structure;

  const Structure * const structure = strData.getStructure();
}

void StoichiometrySearch::init()
{
  initStoichExtents();
}

void StoichiometrySearch::initStoichExtents()
{
  myStoichExtents.setSize(mySpeciesParameters.size());

  for(size_t i = 0; i < mySpeciesParameters.size(); ++i)
  {
    // Need to add one as the extents is a half-open interval i.e. [x0, x1)
    myStoichExtents[i] = mySpeciesParameters[i].maxNum + 1;
  }
}

void StoichiometrySearch::updateTable(
  const common::DataTable::Key &             key,
  const ::sstbx::utility::MultiIdx<size_t> & currentIdx)
{
  using ::boost::lexical_cast;
  using ::std::string;

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

    myTable.insert(key, speciesTableColumn, lexical_cast<string>(numAtomsOfSpecies));

  } // End loop over atoms
}

}
}

