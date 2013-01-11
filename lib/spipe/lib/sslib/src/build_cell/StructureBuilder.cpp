/*
 * StructureBuilder.h
 *
 *  Created on: Nov 10, 2011
 *      Author: Martin Uhrin
 */


// INCLUDES /////////////////
#include "build_cell/StructureBuilder.h"

#include "build_cell/GenerationOutcome.h"
#include "build_cell/IFragmentGenerator.h"
#include "build_cell/IUnitCellGenerator.h"
#include "build_cell/StructureBuild.h"
#include "build_cell/StructureContents.h"
#include "common/Structure.h"

namespace sstbx {
namespace build_cell {

GenerationOutcome
StructureBuilder::generateStructure(common::StructurePtr & structureOut, const common::AtomSpeciesDatabase & speciesDb) const
{
  GenerationOutcome outcome;

  typedef ::std::pair<const IFragmentGenerator *, IFragmentGenerator::GenerationTicket> GeneratorAndTicket;
  ::std::vector<GeneratorAndTicket> generationInfo;
  generationInfo.reserve(myGenerators.size());

  // First find out what the generators want to put in the structure
  StructureContents contents;
  BOOST_FOREACH(const IFragmentGenerator & generator, myGenerators)
  {
    const IFragmentGenerator::GenerationTicket ticket = generator.getTicket();
    generationInfo.push_back(GeneratorAndTicket(&generator, ticket));

    contents += generator.getGenerationContents(ticket, speciesDb);
  }
  // TODO: Sort fragment generators by volume (largest first)

  structureOut.reset(new common::Structure());
  StructureBuild structureBuild(*structureOut, contents);

  // Do we need to create a unit cell?
  if(myUnitCellGenerator.get())
  {
    common::UnitCellPtr cell;
    outcome = myUnitCellGenerator->generateCell(cell, contents);
    
    if(!outcome.success())
      return outcome;

    if(cell.get())
      structureOut->setUnitCell(cell);
    else
    {
      outcome.setFailure("Unit cell generator failed to generate unit cell");
      return outcome;
    }
  }

  BOOST_FOREACH(const GeneratorAndTicket & generatorAndTicket, generationInfo)
  {
    outcome = generatorAndTicket.first->generateFragment(
      structureBuild,
      generatorAndTicket.second,
      speciesDb
    );
    
    if(!outcome.success())
      return outcome;
  }

  // TODO: Check global constraints

  outcome.setSuccess();
  return outcome;
}

void StructureBuilder::setUnitCellGenerator(UnitCellGeneratorPtr unitCellGenerator)
{
  myUnitCellGenerator = unitCellGenerator;
}

const IUnitCellGenerator * StructureBuilder::getUnitCellGenerator() const
{
  return myUnitCellGenerator.get();
}

}
}
