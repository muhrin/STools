/*
 * PotentialGo.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "PotentialGo.h"

#include <locale>
#include <set>

#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/optional.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/tokenizer.hpp>

#include <pipelib/IPipeline.h>

// From SSTbx
#include <build_cell/AtomsDescription.h>
#include <build_cell/StructureDescription.h>
#include <build_cell/StructureDescriptionVisitor.h>
#include <common/Structure.h>
#include <potential/StandardData.h>
#include <potential/IGeomOptimiser.h>
#include <potential/IPotential.h>

#include "common/StructureData.h"
#include "common/SharedData.h"
#include "common/UtilityFunctions.h"

// NAMESPACES ////////////////////////////////


namespace spipe
{
namespace blocks
{

namespace ssbc = ::sstbx::build_cell;
namespace ssc = ::sstbx::common;
namespace ssp = ::sstbx::potential;

class PotentialRadiiSetter : public ssbc::StructureDescriptionVisitor
{
public:
  PotentialRadiiSetter(
    const ssp::IPotential & potential,
    ssc::AtomSpeciesDatabase & speciesDatabase):
  myPotential(potential),
  mySpeciesDatabase(speciesDatabase)
  {}

  virtual void enteringStructure(ssbc::StructureDescription & structureDescription)
  {
    ::std::set<ssc::AtomSpeciesId::Value> species;
    structureDescription.getAtomSpecies(species, ssbc::AtomGroupDescription::DEPTH_ALL_CHILD_GROUPS);
    
    // Now go through asking the potential if it has radii for these species
    ssc::AtomSpeciesId::Value atomSpecies;
    ::boost::optional<double> radius;
    BOOST_FOREACH(atomSpecies, species)
    {
      radius = myPotential.getPotentialRadius(atomSpecies);
      if(radius)
      {
        mySpeciesDatabase.setRadius(atomSpecies, *radius);
      }
    }
  }

  virtual bool visitAtom(ssbc::AtomsDescription & atomDescription)
  {
    // Only set the radius if it hasn't already been set
    if(!atomDescription.getRadius())
    {
      const double * radius = mySpeciesDatabase.getRadius(atomDescription.getSpecies());

      if(radius)
      {
        atomDescription.setRadius(*radius);
      }
    }
    return true;
  }

private:

  const ssp::IPotential &            myPotential;
  ssc::AtomSpeciesDatabase &         mySpeciesDatabase;
};

PotentialGo::PotentialGo(
	const sstbx::potential::IGeomOptimiser & optimiser,
  const ::arma::mat33 * const              externalPressure,
  const bool                               writeOutput):
pipelib::Block<StructureDataTyp, SharedDataTyp>("Potential geometry optimisation"),
myOptimiser(optimiser),
myWriteOutput(writeOutput)
{
  if(externalPressure)
  {
    myExternalPressure = *externalPressure;
  }
  else
  {
    myExternalPressure.fill(0.0);
  }
}

void PotentialGo::pipelineInitialising()
{
  if(myWriteOutput)
  {
    myTableSupport.setFilename(myPipeline->getGlobalData().getOutputFileStem().string() + ".geomopt");
  }
  myTableSupport.registerPipeline(*myPipeline);
}

void PotentialGo::pipelineStarting()
{
  spipe::SharedDataTyp & sharedData = myPipeline->getSharedData();
  ssbc::StructureDescription * const structureDescription = sharedData.getStructureDescription();
  const ssp::IPotential * const potential = myOptimiser.getPotential();
  if(structureDescription && potential)
  {
    // Try to get the potential radii for atoms in the current structure
    updateAtomRadiiFromPotential(*potential, *structureDescription, sharedData.getSpeciesDatabase());
  }
}

void PotentialGo::in(spipe::common::StructureData & data)
{
  ::boost::shared_ptr< sstbx::potential::StandardData<> > optData;
	if(myOptimiser.optimise(*data.getStructure(), optData, &myExternalPressure))
  {
    // Copy over information from the optimisation results
    copyOptimisationResults(*optData.get(), data);

    // Update our data table with the structure data
    updateTable(data);

	  myOutput->in(data);
  }
  else
  {
    // The structure failed to geometry optimise properly so drop it
    myPipeline->dropData(data);
  }
}

void PotentialGo::copyOptimisationResults(
  const sstbx::potential::StandardData<> & optData,
  spipe::common::StructureData & strData)
{
  // Copy over information from the optimisation results

  // Enthaly
  strData.enthalpy.reset(optData.totalEnthalpy);

  // Pressure
  const double pressure = ::arma::trace(optData.stressMtx) / 3.0;
  strData.pressure.reset(pressure);
  strData.objectsStore.insert(common::StructureObjectKeys::PRESSURE_INTERNAL, pressure); 
}

void PotentialGo::updateTable(const ::spipe::StructureDataTyp & strData)
{
  utility::DataTable & table = myTableSupport.getTable();
  const ::std::string & strName = *strData.name;

  if(strData.enthalpy)
    table.insert(strName, "energy", ::boost::lexical_cast< ::std::string>(*strData.enthalpy));
}

void PotentialGo::updateAtomRadiiFromPotential(
  const ssp::IPotential &      potential,
  ssbc::StructureDescription & structureDescription,
  ssc::AtomSpeciesDatabase &   speciesDatabase) const
{
  PotentialRadiiSetter radiiSetter(potential, speciesDatabase);

  structureDescription.traversePreorder(radiiSetter);
}

}}

