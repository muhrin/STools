/*
 * SsLibFactoryYaml.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "spl/factory/SsLibFactoryYaml.h"

#include <memory>

#include <boost/algorithm/string.hpp>
#include <boost/foreach.hpp>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/tokenizer.hpp>

#include "spl/build_cell/AtomsDescription.h"
#include "spl/build_cell/AtomsGroup.h"
#include "spl/build_cell/PointGroups.h"
#include "spl/build_cell/RandomUnitCellGenerator.h"
#include "spl/common/AtomSpeciesDatabase.h"
#include "spl/common/AtomSpeciesId.h"
#include "spl/factory/FactoryError.h"
#include "spl/factory/SsLibElements.h"
#include "spl/io/ResReaderWriter.h"
#include "spl/potential/CastepGeomOptimiser.h"
#include "spl/potential/OptimisationSettings.h"
#include "spl/potential/TpsdGeomOptimiser.h"
#include "spl/potential/Types.h"
#include "spl/utility/IndexingEnums.h"
#include "spl/utility/SortedDistanceComparator.h"

// NAMESPACES ////////////////////////////////

namespace spl {
namespace factory {

namespace fs = ::boost::filesystem;

// Boost Tokenizer stuff
typedef boost::tokenizer< boost::char_separator< char> > Tok;
const boost::char_separator< char> tokSep(" \t");

SsLibFactoryYaml::SsLibFactoryYaml(common::AtomSpeciesDatabase & atomSpeciesDb) :
    myAtomSpeciesDb(atomSpeciesDb), myShapeFactory()
{
}

build_cell::RandomUnitCellPtr
SsLibFactoryYaml::createRandomCellGenerator(const OptionsMap & map) const
{
  using namespace utility::cell_params_enum;

  build_cell::RandomUnitCellPtr cell(new build_cell::RandomUnitCellGenerator());

  {
    const double * const targetVol = map.find(UNIT_CELL_BUILDER_VOLUME);
    if(targetVol)
      cell->setTargetVolume(*targetVol);
  }

  {
    const double * const contentsMultiplier = map.find(
        UNIT_CELL_BUILDER_MULTIPLIER);
    if(contentsMultiplier)
      cell->setContentsMultiplier(*contentsMultiplier);
  }

  {
    const double * const volDelta = map.find(UNIT_CELL_BUILDER_VOLUME_DELTA);
    if(volDelta)
      cell->setVolumeDelta(*volDelta);
  }

  {
    const OptionsMap * const angles = map.find(UNIT_CELL_BUILDER_ANGLES);
    if(angles)
    {
      const double * const min = angles->find(MIN);
      const double * const max = angles->find(MAX);
      if(min)
        cell->setMinAngles(*min);
      if(max)
        cell->setMaxAngles(*max);
    }
  }

  {
    const OptionsMap * const lengths = map.find(UNIT_CELL_BUILDER_ANGLES);
    if(lengths)
    {
      const double * const min = lengths->find(MIN);
      const double * const max = lengths->find(MAX);
      const double * const maxRatio = lengths->find(MAX_RATIO);
      if(min)
        cell->setMinLengths(*min);
      if(max)
        cell->setMaxLengths(*max);
      if(maxRatio)
        cell->setMaxLengthRatio(*maxRatio);
    }
  }

  {
    // Do this after settings the general min/max length/angles as this is a more specific
    // way of specifying the unit cell dimensions
    const ::std::vector< utility::Range< double> > * const abc = map.find(
        UNIT_CELL_BUILDER_ABC);
    if(abc)
    {
      // TODO: Eventuall emit error instead
      SSLIB_ASSERT(abc->size() == 6);

      if(abc->size() == 6)
      {
        for(size_t i = A; i <= GAMMA; ++i)
        {
          cell->setMin(i, (*abc)[i].lower());
          cell->setMax(i, (*abc)[i].upper());
        }
      }
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

  const OptionsMap * const lj = potentialOptions.find(LENNARD_JONES);
  if(lj)
  {
    // Get the species list
    const AtomSpeciesIdVector * const speciesVec = lj->find(SPECIES_LIST);
    if(!speciesVec)
      return pot; // TODO: Emit error

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

    const potential::CombiningRule::Value * const comb = lj->find(
        POT_COMBINING);
    if(!comb)
      return pot;

    const size_t numSpecies = epsilon->n_rows;

    if((numSpecies != sigma->n_rows) || (numSpecies != beta->n_rows))
      return pot;

    pot.reset(
        new potential::SimplePairPotential(myAtomSpeciesDb, *speciesVec,
            *epsilon, *sigma, *cutoff, *beta, (*pow)(0), (*pow)(1), *comb));
  }

  return pot;
}

SsLibFactoryYaml::GeomOptimiserPtr
SsLibFactoryYaml::createGeometryOptimiser(const OptionsMap & optimiserMap) const
{
  GeomOptimiserPtr opt;

  const OptionsMap * const tpsdOptions = optimiserMap.find(TPSD);
  const OptionsMap * const castepOptions = optimiserMap.find(CASTEP);
  if(tpsdOptions)
  {
    const OptionsMap * const potentialOptions = tpsdOptions->find(POTENTIAL);

    // Have to have a potential with this optimiser
    if(!potentialOptions)
      return opt;

    potential::IPotentialPtr potential = createPotential(*potentialOptions);
    if(!potential.get())
      return opt;

    UniquePtr< potential::TpsdGeomOptimiser>::Type tpsd(
        new potential::TpsdGeomOptimiser(potential));

    const double * const tolerance = tpsdOptions->find(TOLERANCE);
    if(tolerance)
      tpsd->setEnergyTolerance(*tolerance);

    opt = tpsd;
  }
  else if(castepOptions)
  {
    // Read in the settings
    potential::CastepGeomOptimiseSettings settings;
    const bool * const keepIntermediates = castepOptions->find(
        CASTEP_KEEP_INTERMEDIATES);
    if(keepIntermediates)
      settings.keepIntermediateFiles = *keepIntermediates;

    const int * const numRoughSteps = castepOptions->find(
        CASTEP_NUM_ROUGH_STEPS);
    if(numRoughSteps)
      settings.numRoughSteps = *numRoughSteps;

    const int * const numSelfConsistent = castepOptions->find(
        CASTEP_NUM_SELF_CONSISTENT);
    if(numSelfConsistent)
      settings.numConsistentRelaxations = *numSelfConsistent;

    const ::std::string * const castepExe = castepOptions->find(CASTEP_EXE);
    const ::std::string * const seed = castepOptions->find(CASTEP_SEED);
    if(castepExe && seed)
      opt.reset(
          new potential::CastepGeomOptimiser(*castepExe, *seed, settings));
  }

  return opt;
}

potential::OptimisationSettings
SsLibFactoryYaml::createOptimisationSettings(const OptionsMap & options) const
{
  potential::OptimisationSettings settings;

  settings.maxIter = toOptional(options.find(MAX_ITER));

  const double * const pressure = options.find(PRESSURE);
  if(pressure)
  {
    settings.pressure.reset(::arma::zeros(3, 3));
    settings.pressure->diag().fill(*pressure);
  }

  return settings;
}

utility::IStructureComparatorPtr
SsLibFactoryYaml::createStructureComparator(const OptionsMap & map) const
{
  utility::IStructureComparatorPtr comparator;

  const OptionsMap * const comparatorMap = map.find(SORTED_DISTANCE);
  if(comparatorMap)
  {
    utility::SortedDistanceComparator::ConstructionInfo constructInfo;

    {
      const double * const tolerance = map.find(TOLERANCE);
      if(tolerance)
        constructInfo.tolerance = *tolerance;
    }
    {
      const bool * const volAgnostic = map.find(
          SORTED_DISTANCE__VOLUME_AGNOSTIC);
      if(volAgnostic)
        constructInfo.volumeAgnostic = *volAgnostic;
    }
    {
      const bool * const usePrimitive = map.find(
          SORTED_DISTANCE__USE_PRIMITIVE);
      if(usePrimitive)
        constructInfo.usePrimitive = *usePrimitive;
    }
    comparator.reset(new utility::SortedDistanceComparator(constructInfo));
  }

  return comparator;
}

SsLibFactoryYaml::StructureContentType::Value
SsLibFactoryYaml::getStructureContentType(
    const AtomsDataEntry & atomsEntry) const
{
  // If it is a list then we know it's a compact atoms info object
  if(::boost::get< AtomsCompactInfo>(&atomsEntry))
    return StructureContentType::ATOMS;

  const OptionsMap * const map = ::boost::get< OptionsMap>(&atomsEntry);
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
SsLibFactoryYaml::createAtomsDescription(const AtomsDataEntry & atomsEntry,
    const io::AtomFormatParser & parser) const
{
  build_cell::AtomsDescriptionPtr atomsDescription;

  ::boost::optional< AtomSpeciesCount> speciesAndCount = parser.getValue(
      SPECIES, atomsEntry);
  if(!speciesAndCount
      || (speciesAndCount->count.nullSpan()
          && speciesAndCount->count.lower() == 0))
    return atomsDescription;

  atomsDescription.reset(
      new build_cell::AtomsDescription(speciesAndCount->species,
          speciesAndCount->count));

  const OptionalDouble radius = parser.getValue(RADIUS, atomsEntry);
  if(radius)
    atomsDescription->setRadius(*radius);

  const OptionalArmaVec3 pos = parser.getValue(POSITION, atomsEntry);
  if(pos)
    atomsDescription->setPosition(*pos);

  return atomsDescription;
}

build_cell::StructureBuilderPtr
SsLibFactoryYaml::createStructureBuilder(const OptionsMap & map) const
{
  build_cell::StructureBuilder::ConstructInfo constructInfo;

  { // Atoms overlap
    const double * const atomsOverlap = map.find(ATOMS_OVERLAP);
    if(atomsOverlap)
      constructInfo.atomsOverlap = *atomsOverlap;
  }

  { // Cluster mode
    const bool * const isCluster = map.find(CLUSTER);
    if(isCluster)
      constructInfo.isCluster = *isCluster;
  }

  build_cell::StructureBuilderPtr builder(
      new build_cell::StructureBuilder(constructInfo));

  io::AtomFormatParser atomsFormatParser;
  {
    const io::AtomFormatParser::FormatDescription * const format = map.find(
        ATOMS_FORMAT);
    if(format)
      atomsFormatParser.setFormat(*format);
  }

  {
    const double * const atomsRadius = map.find(RADIUS);
    if(atomsRadius)
      atomsFormatParser.setDefault(RADIUS, *atomsRadius);
  }

  // Generators
  const AtomsDataEntryList * const atoms = map.find(ATOMS);
  if(atoms)
  {
    // Try creating the default atoms generator
    {
      build_cell::AtomsGroupPtr atomsGenerator = createAtomsGroup(map,
          atomsFormatParser);
      if(atomsGenerator.get())
        builder->addGenerator(atomsGenerator);
    }

    // Now look for other generators
    BOOST_FOREACH(const AtomsDataEntry & atomsEntry, *atoms)
    {
      if(getStructureContentType(atomsEntry) == StructureContentType::GROUP)
      {
        const OptionsMap * const groupOptions = ::boost::get< OptionsMap>(
            atomsEntry).find(ATOMS_GROUP);
        build_cell::AtomsGroupPtr atomsGenerator = createAtomsGroup(
            *groupOptions, atomsFormatParser);
        if(atomsGenerator.get())
          builder->addGenerator(atomsGenerator);
      }
    }
  }

  // Unit cell
  const OptionsMap * unitCellBuilder = map.find(UNIT_CELL_BUILDER);
  if(unitCellBuilder)
  {
    build_cell::IUnitCellGeneratorPtr ucGen(
        createRandomCellGenerator(*unitCellBuilder));
    if(ucGen.get())
      builder->setUnitCellGenerator(ucGen);
  }

  // Symmetry
  const OptionsMap * const symmetry = map.find(SYMMETRY);
  if(symmetry)
  {
    const ::std::string * const pointGroup = symmetry->find(POINT_GROUP);
    if(pointGroup)
    {
      build_cell::PointGroup group;
      if(build_cell::getPointGroup(group, *pointGroup))
        builder->setPointGroup(group);
    }
  }

  return builder;
}

build_cell::AtomsGroupPtr
SsLibFactoryYaml::createAtomsGroup(const OptionsMap & map,
    io::AtomFormatParser & parser) const
{
  build_cell::AtomsGroupPtr atomsGenerator(new build_cell::AtomsGroup());

  // Try creating a generator shape
  UniquePtr< build_cell::IGeneratorShape>::Type genShape;
  myShapeFactory.createShape(genShape, map);

  const int * const num = map.find(NUM);
  const ::arma::vec3 * const pos = map.find(POSITION);
  const ::arma::vec4 * const rot = map.find(ROT_AXIS_ANGLE);

  if(num)
    atomsGenerator->setNumReplicas(*num);

  if(pos)
    atomsGenerator->setPosition(*pos);
  else
    atomsGenerator->setTransformMode(
        atomsGenerator->getTransformMode()
            | build_cell::AtomsGroup::TransformMode::RAND_POS);

  if(rot)
    atomsGenerator->setRotation(*rot);
  else
  {
    atomsGenerator->setTransformMode(
        atomsGenerator->getTransformMode()
            | build_cell::AtomsGroup::TransformMode::RAND_ROT_DIR
            | build_cell::AtomsGroup::TransformMode::RAND_ROT_ANGLE);
  }

  // Check if there is a 'global' radius
  {
    const double * const radius = map.find(RADIUS);
    if(radius)
      parser.setDefault(RADIUS, *radius);
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
        build_cell::AtomsDescriptionPtr atomsDescription =
            createAtomsDescription(atomData, parser);
        if(atomsDescription.get())
          atomsGenerator->insertAtoms(*atomsDescription);
      }
    }
  }

  // Species distances
  typedef ::std::map< ::std::string, double> PairDistances;
  const PairDistances * const pairDistances = map.find(PAIR_DISTANCES);
  if(pairDistances)
  {
    ::std::vector< ::std::string> species;
    BOOST_FOREACH(PairDistances::const_reference entry, *pairDistances)
    {
      ::boost::split(species, entry.first, ::boost::is_any_of("~"));
      if(species.size() == 2)
        atomsGenerator->addSpeciesPairDistance(
            build_cell::SpeciesPair(species[0], species[1]), entry.second);
    }
  }

  return atomsGenerator;
}

}
}

