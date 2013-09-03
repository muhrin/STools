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

#include <spl/build_cell/AtomsDescription.h>
#include <spl/build_cell/AtomsGenerator.h>
#include <spl/build_cell/BuildCellFwd.h>
#include <spl/build_cell/IFragmentGenerator.h>
#include <spl/build_cell/IStructureGenerator.h>
#include <spl/build_cell/StructureBuilder.h>
#include <spl/common/AtomSpeciesDatabase.h>
#include <spl/common/Structure.h>
#include <spl/io/BoostFilesystem.h>
#include <spl/io/ResourceLocator.h>
#include <spl/utility/MultiIdx.h>

// Local includes
#include "common/PipeFunctions.h"
#include "common/StructureData.h"
#include "common/SharedData.h"
#include "common/UtilityFunctions.h"
#include "utility/DataTable.h"

namespace spipe {
namespace blocks {

// NAMESPACE ALIASES /////////////////////////
namespace fs = ::boost::filesystem;
namespace common = ::spipe::common;
namespace ssbc = ::spl::build_cell;
namespace ssc = ::spl::common;
namespace ssio = ::spl::io;
namespace ssu = ::spl::utility;
namespace structure_properties = ssc::structure_properties;

SearchStoichiometries::SearchStoichiometries(
    const ::spl::common::AtomSpeciesId::Value species1,
    const ::spl::common::AtomSpeciesId::Value species2, const size_t maxAtoms,
    BlockHandle & subpipe, StructureBuilderPtr structureBuilder) :
    Block("Sweep stoichiometry"), myMaxAtoms(maxAtoms), mySubpipe(subpipe), myStructureGenerator(
        structureBuilder)
{
  mySpeciesParameters.push_back(SpeciesParameter(species1, maxAtoms));
  mySpeciesParameters.push_back(SpeciesParameter(species2, maxAtoms));
}

SearchStoichiometries::SearchStoichiometries(
    const SpeciesParameters & speciesParameters, const size_t maxAtoms,
    const double atomsRadius, BlockHandle & sweepPipe,
    StructureBuilderPtr structureBuilder) :
    Block("Sweep stoichiometry"), mySpeciesParameters(speciesParameters), myMaxAtoms(
        maxAtoms), mySubpipe(sweepPipe), myTableSupport(fs::path("stoich.dat")), myStructureGenerator(
        structureBuilder)
{
}

void
SearchStoichiometries::pipelineInitialising()
{
  myTableSupport.setFilename(
      common::getOutputFileStem(getEngine()->sharedData(),
          getEngine()->globalData()) + ".stoich");
  myTableSupport.registerEngine(getEngine());
}

void
SearchStoichiometries::pipelineStarting()
{
  // Save where the pipeline will be writing to
  myOutputPath = getEngine()->sharedData().getOutputPath();
}

void
SearchStoichiometries::start()
{
  using ::boost::lexical_cast;
  using ::std::string;

  const ssc::AtomSpeciesDatabase & atomsDb =
      getEngine()->globalData().getSpeciesDatabase();

  // Start looping over the possible stoichiometries
  size_t totalAtoms = 0;
  ::std::string sweepPipeOutputPath;

  const ssu::MultiIdxRange< unsigned int> stoichRange = getStoichRange();
  BOOST_FOREACH(const ssu::MultiIdx<unsigned int> & currentIdx, stoichRange)
  {
    // Need to get the shared data each time as it may have been reset after each
    // run of the pipe
    SharedDataType & sweepPipeData = mySubpipeEngine->sharedData();

    totalAtoms = currentIdx.sum();
    if(totalAtoms == 0 || totalAtoms > myMaxAtoms)
      continue;

    // Create a new structure description
    StructureBuilderPtr builder = newStructureGenerator();

    // Insert all the atoms
    ::std::stringstream stoichStringStream;
    size_t numAtomsOfSpecies;
    ssc::AtomSpeciesId::Value species;
    for(size_t i = 0; i < currentIdx.dims(); ++i)
    {
      species = mySpeciesParameters[i].id;
      numAtomsOfSpecies = currentIdx[i];

      if(numAtomsOfSpecies > 0)
      {
        ::spl::UniquePtr< ssbc::AtomsGenerator>::Type atomsGenerator(
            new ssbc::AtomsGenerator());
        atomsGenerator->insertAtoms(
            ssbc::AtomsDescription(mySpeciesParameters[i].id,
                numAtomsOfSpecies));
        builder->addGenerator(atomsGenerator);
      }

      stoichStringStream << numAtomsOfSpecies;

      // Append the species symbol
      if(!species.empty())
        stoichStringStream << species;

      // Add delimiter apart from for last species
      if(i + 1 < currentIdx.dims())
        stoichStringStream << "-";

    } // End loop over atoms

    // Transfer ownership to the pipeline
    sweepPipeData.setStructureGenerator(builder);

    // Append the species ratios to the output directory name
    sweepPipeData.appendToOutputDirName(stoichStringStream.str());

    // Find out the pipeline relative path to where all the structures are going to be saved
    sweepPipeOutputPath = sweepPipeData.getPipeRelativeOutputPath().string();

    // Start the sweep pipeline
    mySubpipeEngine->run();

    // Update the table
    updateTable(sweepPipeOutputPath, currentIdx, atomsDb);

    // Send any finished structure data down my pipe, this will also
    // update the table with any information from the buffered structures
    releaseBufferedStructures(sweepPipeOutputPath);

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
    lastSavedRelative = strData->getRelativeSavePath(getEngine()->sharedData().getOutputPath());

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
          ::boost::lexical_cast< ::std::string>(*spacegroup));
    }

    // Try to calculate the energy/atom
    internalEnergy = structure->getProperty(
        structure_properties::general::ENERGY_INTERNAL);
    if(internalEnergy)
    {
      const size_t numAtoms = structure->getNumAtoms();
      if(numAtoms != 0)
        table.insert(tableKey, "energy/atom",
            common::getString(*internalEnergy / numAtoms));
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
  const size_t numSpecies = mySpeciesParameters.size();

  ssu::MultiIdx< unsigned int> maxSpecies(numSpecies);

  for(size_t i = 0; i < numSpecies; ++i)
  {
    // Need to add one as the extents is a half-open interval i.e. [x0, x1)
    maxSpecies[i] = mySpeciesParameters[i].maxNum + 1;
  }

  return ssu::MultiIdxRange< unsigned int>(
      ssu::MultiIdx< unsigned int>(numSpecies), maxSpecies);
}

void
SearchStoichiometries::updateTable(const utility::DataTable::Key & key,
    const ssu::MultiIdx< unsigned int> & currentIdx,
    const ssc::AtomSpeciesDatabase & atomsDb)
{
  using ::boost::lexical_cast;
  using ::std::string;

  utility::DataTable & table = myTableSupport.getTable();

  ssc::AtomSpeciesId::Value species;
  string speciesTableColumn;
  size_t numAtomsOfSpecies;
  for(size_t i = 0; i < currentIdx.dims(); ++i)
  {
    species = mySpeciesParameters[i].id;
    numAtomsOfSpecies = currentIdx[i];

    if(!species.empty())
      speciesTableColumn = species;
    else
      speciesTableColumn = lexical_cast< string>(i);

    table.insert(key, speciesTableColumn,
        lexical_cast< string>(numAtomsOfSpecies));

  } // End loop over atoms
}

SearchStoichiometries::StructureBuilderPtr
SearchStoichiometries::newStructureGenerator() const
{
  if(myStructureGenerator.get())
    return StructureBuilderPtr(
        new ssbc::StructureBuilder(*myStructureGenerator));
  else
    return StructureBuilderPtr(new ssbc::StructureBuilder());
}

}
}

