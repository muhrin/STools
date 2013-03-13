/*
 * SsLibFactoryYaml.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "SSLib.h"

#include "factory/SsLibFactoryYaml.h"

#include <memory>

#include <boost/algorithm/string.hpp>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/tokenizer.hpp>

// Local includes
#include "build_cell/AtomsDescription.h"
#include "build_cell/AtomsGenerator.h"
#include "build_cell/RandomUnitCellGenerator.h"
#include "common/AtomSpeciesDatabase.h"
#include "common/AtomSpeciesId.h"
#include "factory/FactoryError.h"
#include "factory/SsLibElements.h"
#include "io/ResReaderWriter.h"
#include "potential/TpsdGeomOptimiser.h"
#include "potential/Types.h"
#include "utility/IndexingEnums.h"
#include "utility/SortedDistanceComparator.h"


// NAMESPACES ////////////////////////////////

namespace sstbx {
namespace factory {

// Boost Tokenizer stuff
typedef boost::tokenizer<boost::char_separator<char> > Tok;
const boost::char_separator<char> tokSep(" \t");

SsLibFactoryYaml::SsLibFactoryYaml(common::AtomSpeciesDatabase & atomSpeciesDb):
myAtomSpeciesDb(atomSpeciesDb)
{}
//
//common::StructurePtr
//SsLibFactoryYaml::createStructure(const YAML::Node & structureNode) const
//{
//  common::StructurePtr structure(new common::Structure());
//
//
//  // TODO: Use StructureYamlGenerator here
//
//  return structure;
//}
//
//common::UnitCellPtr
//SsLibFactoryYaml::createUnitCell(const OptionsMap & map) const
//{
//  namespace param = utility::cell_params_enum;
//
//  common::UnitCellPtr cell;
//
//  const ::std::vector<double> * const abc = map.find(ABC);
//  if(abc)
//  {
//    if(!(abc->size() == 6))
//      return cell; // TODO: Emit error
//
//    cell.reset(new common::UnitCell(
//      (*abc)[param::A],
//      (*abc)[param::B],
//      (*abc)[param::C],
//      (*abc)[param::ALPHA],
//      (*abc)[param::BETA],
//      (*abc)[param::GAMMA])
//    );
//  }
//
//  return cell;
//}

build_cell::RandomUnitCellPtr
SsLibFactoryYaml::createRandomCellGenerator(const OptionsMap & map) const
{
  using namespace utility::cell_params_enum;

  build_cell::RandomUnitCellPtr cell(new build_cell::RandomUnitCellGenerator());

  {
    const ::std::vector<double> * const abc = map.find(UNIT_CELL_BUILDER_ABC);
    if(abc)
    {
      if(abc->size() == 6)
      {
        for(size_t i = A; i <= GAMMA; ++i)
        {
          cell->setMin(i, (*abc)[i]);
          cell->setMax(i, (*abc)[i]);
        }
      }
    }
  }

  {
    const double * const targetVol = map.find(UNIT_CELL_BUILDER_VOLUME);
    if(targetVol)
      cell->setTargetVolume(*targetVol);
  }

  {
    const MinMax * const angles = map.find(UNIT_CELL_BUILDER_ANGLES);
    if(angles)
    {
      if(angles->min)
        cell->setMinAngles(*angles->min);
      if(angles->max)
        cell->setMaxAngles(*angles->max);
    }
  }

  {
    const MinMax * const lengths = map.find(UNIT_CELL_BUILDER_ANGLES);
    if(lengths)
    {
      if(lengths->min)
        cell->setMinLengths(*lengths->min);
      if(lengths->max)
        cell->setMaxLengths(*lengths->max);
    }
  }

  return cell;
}

build_cell::IStructureGeneratorPtr
SsLibFactoryYaml::createStructureGenerator(const OptionsMap & map) const
{
  build_cell::IStructureGeneratorPtr generator;

  const OptionsMap * generatorMap = map.find(BUILDER);
  if(generatorMap)
  {
    generator = createStructureBuilder(*generatorMap);
  }

  return generator;
}

potential::IPotentialPtr
SsLibFactoryYaml::createPotential(const OptionsMap & potentialOptions) const
{
  potential::IPotentialPtr pot;

  const OptionsMap * const lj  = potentialOptions.find(LENNARD_JONES);
  if(lj)
  {

    // Get the species list
    const AtomSpeciesIdVector * const speciesVec = lj->find(SPECIES_LIST);
    if(!speciesVec)
      return pot; // TODO: Emit error

    ::std::vector<common::AtomSpeciesId::Value> species;
    species.reserve(speciesVec->size());
    BOOST_FOREACH(const ::std::string & spec, *speciesVec)
    {
      species.push_back(myAtomSpeciesDb.getIdFromSymbol(spec));
    }

    // Build up the potential parameters one by one
    const ::arma::mat * const epsilon = lj->find(LJ_EPSILON);
    if(!epsilon)
      return pot;

    const ::arma::mat * const sigma = lj->find(LJ_SIGMA);
    if(!sigma)
      return pot;

    const ::arma::mat * const beta = lj->find(LJ_BETA);
    if(!beta)
      return pot;

    const ::arma::vec * const pow = lj->find(LJ_POWERS);
    if(!pow)
      return pot;

    const double * const cutoff = lj->find(CUTOFF);
    if(!cutoff)
      return pot;

    const potential::CombiningRule::Value * const comb = lj->find(POT_COMBINING);
    if(!comb)
      return pot;

    const size_t numSpecies = epsilon->n_rows;

    if((numSpecies != sigma->n_rows) || (numSpecies != beta->n_rows))
      return pot;

    pot.reset(new potential::SimplePairPotential(
      myAtomSpeciesDb,
      species,
      *epsilon,
      *sigma,
      *cutoff,
      *beta,
      (*pow)(0),
      (*pow)(1),
      *comb
    ));
  }

  return pot;
}



SsLibFactoryYaml::GeomOptimiserPtr
SsLibFactoryYaml::createGeometryOptimiser(
  const OptionsMap & optimiserMap,
  const OptionsMap * potentialMap
) const
{
  GeomOptimiserPtr opt;

  const OptionsMap * optimiser = optimiserMap.find(TPSD);
  if(optimiser)
  {
    // Have to have a potential with this optimiser
    if(!potentialMap)
      return opt; // TODO: Emit error
    potential::IPotentialPtr potential = createPotential(*potentialMap);
    if(!potential.get())
      return opt; // TODO: Emit error

    const double * const tolerance = optimiserMap.find(TOLERANCE);
    
    UniquePtr<potential::TpsdGeomOptimiser>::Type tpsd(new potential::TpsdGeomOptimiser(potential));
    if(tolerance)
      tpsd->setTolerance(*tolerance);

    opt = tpsd;      
  }

  return opt;
}

utility::IStructureComparatorPtr
SsLibFactoryYaml::createStructureComparator(const OptionsMap & map) const
{
  utility::IStructureComparatorPtr comparator;

  const OptionsMap * const comparatorMap = map.find(SORTED_DISTANCE);
  if(comparatorMap)
  {
    const double  * const tolerance = map.find(TOLERANCE);
    const bool * const volAgnostic = map.find(SORTED_DISTANCE__VOLUME_AGNOSTIC);
    const bool * const usePrimitive = map.find(SORTED_DISTANCE__USE_PRIMITIVE);
    
    // Create with the given options (if any)
    if(tolerance && volAgnostic && usePrimitive)
      comparator.reset(new utility::SortedDistanceComparator(*tolerance, *volAgnostic, *usePrimitive));
    else if(tolerance && volAgnostic)
      comparator.reset(new utility::SortedDistanceComparator(*tolerance, *volAgnostic));
    else if(tolerance)
      comparator.reset(new utility::SortedDistanceComparator(*tolerance));
    else
      comparator.reset(new utility::SortedDistanceComparator());
  }

  return comparator;
}
//
//SsLibFactoryYaml::UniqueStructureSetPtr
//SsLibFactoryYaml::createStructureSet(const YAML::Node & node)
//{
//  //// Make sure we have a structure set node
//  //if(node.Scalar() != kw::STR_SET)
//  //{
//  //  throw FactoryError() << ErrorType(BAD_TAG) << NodeName(node.Scalar());
//  //}
//
//  UniqueStructureSetPtr strSet;
//
//  // First try to create the comparator
//  if(!node[kw::COMPARATOR])
//    return strSet;
//
//  utility::IStructureComparatorPtr comparator = createStructureComparator(node[kw::COMPARATOR]);
//
//  if(comparator.get())
//    strSet.reset(new ssu::UniqueStructureSet<>(comparator));
//  
//  return strSet;
//}

//::sstbx::io::IStructureWriter *
//SsLibFactoryYaml::createStructureWriter(const YAML::Node & node)
//{
//  //// Make sure we have a structure writer tag
//  //if(node.Scalar() != kw::STR_SET)
//  //{
//  //  throw FactoryError() << ErrorType(BAD_TAG) << NodeName(node.Scalar());
//  //}
//
//  // Check we have the required keywords
//  checkKeyword(kw::TYPE, node);
//
//  const ::std::string type = node[kw::TYPE].as< ::std::string>();
//
//  ssio::IStructureWriter * writer = NULL;
//  if(type == kw::STR_WRITER__TYPE___RES)
//  {
//    writer = new ssio::ResReaderWriter();
//  }
//
//  if(writer)
//    myStructureWriters.push_back(writer);
//
//  return writer;
//}


SsLibFactoryYaml::StructureContentType::Value
SsLibFactoryYaml::getStructureContentType(const AtomsDataEntry & atomsEntry) const
{
  // If it is a list then we know it's a compact atoms info object
  if(::boost::get<AtomsCompactInfo>(&atomsEntry))
    return StructureContentType::ATOMS;

  const OptionsMap * const map = ::boost::get<OptionsMap>(&atomsEntry);
  if(map)
  {
    if(map->find(SPECIES))
      return StructureContentType::ATOMS;
    else if(map->find(ATOMS_GROUP))
      return StructureContentType::GROUP;
  }

  return StructureContentType::UNKNOWN;
}

build_cell::AtomsDescriptionPtr
SsLibFactoryYaml::createAtomsDescription(
  const AtomsDataEntry & atomsEntry,
  const io::AtomFormatParser & parser) const
{
  build_cell::AtomsDescriptionPtr atomsDescription;

  ::boost::optional<AtomSpeciesCount> speciesAndCount = parser.getValue(SPECIES, atomsEntry);
  if(!speciesAndCount)
    return atomsDescription;

  const common::AtomSpeciesId::Value species =
    myAtomSpeciesDb.getIdFromSymbol(speciesAndCount->species);
  atomsDescription.reset(new build_cell::AtomsDescription(species, speciesAndCount->count));

  ::boost::optional<double> radius = parser.getValue(ATOM_RADIUS, atomsEntry);
  if(radius)
    atomsDescription->setRadius(*radius);

  return atomsDescription;
}

build_cell::StructureBuilderPtr
SsLibFactoryYaml::createStructureBuilder(const OptionsMap & map) const
{
  build_cell::StructureBuilderPtr builder(new build_cell::StructureBuilder());

  io::AtomFormatParser atomsFormatParser;
  {
    const io::AtomFormatParser::FormatDescription * const format = map.find(ATOMS_FORMAT);
    if(format)
      atomsFormatParser.setFormat(*format);
  }

  {
    const double * const atomsRadius = map.find(ATOM_RADIUS);
    if(atomsRadius)
      atomsFormatParser.setDefault(ATOM_RADIUS, *atomsRadius);
  }

  // Generators
  const AtomsDataEntryList * const atoms = map.find(ATOMS);
  if(atoms)
  {
    // Try creating the default atoms generator
    {
      build_cell::AtomsGeneratorPtr atomsGenerator = createAtomsGenerator(map, atomsFormatParser);
      if(atomsGenerator.get())
        builder->addGenerator(atomsGenerator);
    }

    // Now look for other generators
    BOOST_FOREACH(const AtomsDataEntry & atomsEntry, *atoms)
    {
      if(getStructureContentType(atomsEntry) == StructureContentType::GROUP)
      {
        const OptionsMap * const groupOptions = ::boost::get<OptionsMap>(atomsEntry).find(ATOMS_GROUP);
        build_cell::AtomsGeneratorPtr atomsGenerator = createAtomsGenerator(*groupOptions, atomsFormatParser);
        if(atomsGenerator.get())
          builder->addGenerator(atomsGenerator);
      }
    }
  }

  // Unit cell
  const OptionsMap * unitCellBuilder = map.find(UNIT_CELL_BUILDER);
  if(unitCellBuilder)
  {
    build_cell::IUnitCellGeneratorPtr ucGen(createRandomCellGenerator(*unitCellBuilder));
    if(ucGen.get())
      builder->setUnitCellGenerator(ucGen);
  }

  return builder;
}

build_cell::AtomsGeneratorPtr
SsLibFactoryYaml::createAtomsGenerator(
  const OptionsMap & map,
  io::AtomFormatParser & parser
) const
{
  build_cell::AtomsGeneratorPtr atomsGenerator(new build_cell::AtomsGenerator());

  // Check if there is a 'global' radius
  {
    const double * const radius = map.find(ATOM_RADIUS);
    if(radius)
      parser.setDefault(ATOM_RADIUS, *radius);
  }

  ///////////
  // Atoms
  const AtomsDataEntryList * const atomsList = map.find(ATOMS);
  if(atomsList)
  {
      BOOST_FOREACH(const AtomsDataEntry & atomData, *atomsList)
      {
        if(getStructureContentType(atomData) == StructureContentType::ATOMS)
        {
          build_cell::AtomsDescriptionPtr atomsDescription = createAtomsDescription(atomData, parser);
          if(atomsDescription.get())
            atomsGenerator->addAtoms(*atomsDescription);
        }
      }
  }

  // Try creating a generator shape
  UniquePtr<build_cell::IGeneratorShape>::Type shape;
  myShapeFactory.createShape(shape, map);
  if(shape.get())
    atomsGenerator->setGeneratorShape(shape);

  return atomsGenerator;
}

}
}


