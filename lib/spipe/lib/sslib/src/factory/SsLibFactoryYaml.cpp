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
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/tokenizer.hpp>

// Local includes
#include "build_cell/AtomsDescription.h"
#include "build_cell/AtomsGenerator.h"
#include "build_cell/PointGroups.h"
#include "build_cell/RandomUnitCellGenerator.h"
#include "common/AtomSpeciesDatabase.h"
#include "common/AtomSpeciesId.h"
#include "factory/FactoryError.h"
#include "factory/SsLibElements.h"
#include "io/ResReaderWriter.h"
#include "potential/CastepGeomOptimiser.h"
#include "potential/TpsdGeomOptimiser.h"
#include "potential/Types.h"
#include "utility/IndexingEnums.h"
#include "utility/SortedDistanceComparator.h"


// NAMESPACES ////////////////////////////////

namespace sstbx {
namespace factory {

namespace fs = ::boost::filesystem;

// Boost Tokenizer stuff
typedef boost::tokenizer<boost::char_separator<char> > Tok;
const boost::char_separator<char> tokSep(" \t");

SsLibFactoryYaml::SsLibFactoryYaml(common::AtomSpeciesDatabase & atomSpeciesDb):
myAtomSpeciesDb(atomSpeciesDb),
myShapeFactory()
{}

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
    const double * const contentsMultiplier = map.find(UNIT_CELL_BUILDER_MULTIPLIER);
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
    const ::std::vector<utility::Range<double> > * const abc = map.find(UNIT_CELL_BUILDER_ABC);
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
  const OptionsMap * potentialMap,
  const OptionsMap * globalOptions
) const
{
  GeomOptimiserPtr opt;

  const OptionsMap * const tpsdOptions = optimiserMap.find(TPSD);
  const OptionsMap * const castepOptions = optimiserMap.find(CASTEP);
  if(tpsdOptions)
  {
    // Have to have a potential with this optimiser
    if(!potentialMap)
      return opt; // TODO: Emit error
    potential::IPotentialPtr potential = createPotential(*potentialMap);
    if(!potential.get())
      return opt; // TODO: Emit error

    const double * const tolerance = tpsdOptions->find(TOLERANCE);
    
    UniquePtr<potential::TpsdGeomOptimiser>::Type tpsd(new potential::TpsdGeomOptimiser(potential));
    if(tolerance)
      tpsd->setTolerance(*tolerance);

    opt = tpsd;      
  }
  else if(castepOptions)
  {
    const ::std::string * const castepExe = find(CASTEP_EXE, *castepOptions, globalOptions);
    const ::std::string * const seed = castepOptions->find(CASTEP_SEED);

    // Read in the settings
    potential::CastepGeomOptimiseSettings settings;
    const bool * const keepIntermediates = castepOptions->find(CASTEP_KEEP_INTERMEDIATES);
    const int * const numRoughSteps = castepOptions->find(CASTEP_NUM_ROUGH_STEPS);
    const int * const numSelfConsistent = castepOptions->find(CASTEP_NUM_SELF_CONSISTENT);
    if(keepIntermediates)
      settings.keepIntermediateFiles = *keepIntermediates;
    if(numRoughSteps)
      settings.numRoughSteps = *numRoughSteps;
    if(numSelfConsistent)
      settings.numConsistentRelaxations = *numSelfConsistent;


    if(castepExe && keepIntermediates && seed)
      opt.reset(new potential::CastepGeomOptimiser(*castepExe, *seed, settings));
    else
    {
      // TODO: Emit error
    }
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

  const common::AtomSpeciesId::Value species = myAtomSpeciesDb.getIdFromSymbol(speciesAndCount->species);
  atomsDescription.reset(new build_cell::AtomsDescription(species, speciesAndCount->count));

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
  build_cell::StructureBuilderPtr builder(new build_cell::StructureBuilder());

  io::AtomFormatParser atomsFormatParser;
  {
    const io::AtomFormatParser::FormatDescription * const format = map.find(ATOMS_FORMAT);
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

  // Cluster mode
  const bool * const clusterMode = map.find(CLUSTER);
  if(clusterMode)
    builder->setCluster(*clusterMode);

  return builder;
}

build_cell::AtomsGeneratorPtr
SsLibFactoryYaml::createAtomsGenerator(
  const OptionsMap & map,
  io::AtomFormatParser & parser
) const
{
  build_cell::AtomsGeneratorConstructionInfo constructInfo;
  
  // Try creating a generator shape
  myShapeFactory.createShape(constructInfo.genShape, map);
  
  const int * const num = map.find(NUM);
  const ::arma::vec3 * const pos = map.find(POSITION);
  const ::arma::vec4 * const rot = map.find(ROT_AXIS_ANGLE);

  if(num)
    constructInfo.numReplicas = *num;

  if(pos)
    constructInfo.pos.reset(*pos);
  else
    constructInfo.transformMask |= build_cell::AtomsGenerator::TransformSettings::RAND_POS;

  if(rot)
    constructInfo.rot.reset(*rot);
  else
  {
    constructInfo.transformMask |=
      build_cell::AtomsGenerator::TransformSettings::RAND_ROT_DIR |
      build_cell::AtomsGenerator::TransformSettings::RAND_ROT_ANGLE;
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
        build_cell::AtomsDescriptionPtr atomsDescription = createAtomsDescription(atomData, parser);
        if(atomsDescription.get())
          constructInfo.atoms.push_back(*atomsDescription);
      }
    }
  }

  build_cell::AtomsGeneratorPtr atomsGenerator(new build_cell::AtomsGenerator(constructInfo));

  return atomsGenerator;
}

}
}


