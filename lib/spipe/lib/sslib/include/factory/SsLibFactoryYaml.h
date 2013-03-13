/*
 * SsLibFactoryYaml.h
 *
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef SS_LIB_FACTORY_YAML_H
#define SS_LIB_FACTORY_YAML_H

// INCLUDES /////////////////////////////////////////////
#include "SSLib.h"

#include <map>

#include <boost/noncopyable.hpp>
#include <boost/optional.hpp>
#include <boost/ptr_container/ptr_vector.hpp>
#include <boost/utility.hpp>

// Local includes
#include "OptionalTypes.h"
#include "build_cell/IStructureGenerator.h"
#include "build_cell/StructureBuilder.h"
#include "build_cell/BuildCellFwd.h"
#include "factory/FactoryFwd.h"
#include "factory/GenShapeFactory.h"
#include "io/AtomFormatParser.h"
#include "io/IStructureWriter.h"
#include "potential/Types.h"
#include "utility/HeterogeneousMapKey.h"
#include "utility/IStructureComparator.h"
#include "utility/UniqueStructureSet.h"


// FORWARD DECLARATIONS ////////////////////////////////////

namespace sstbx {
namespace common {
class AtomSpeciesDatabase;
}

namespace factory {

class SsLibFactoryYaml : ::boost::noncopyable
{
  typedef utility::HeterogeneousMap OptionsMap;
public:
  typedef UniquePtr<potential::IGeomOptimiser>::Type GeomOptimiserPtr;
  typedef UniquePtr<utility::UniqueStructureSet<> >::Type UniqueStructureSetPtr;

  enum ErrorCode
  {
    UNKNOWN,
    BAD_TAG,
    UNRECOGNISED_KEYWORD,
    REQUIRED_KEYWORD_MISSING,
    MALFORMED_VALUE,
    SEQUENCE_LENGTH_INVALID
  };

  SsLibFactoryYaml(common::AtomSpeciesDatabase & atomSpeciesDb);

  //common::StructurePtr                  createStructure(const YAML::Node & structureNode) const;
  //common::UnitCellPtr                   createUnitCell(const OptionsMap & map) const;
  
  GeomOptimiserPtr createGeometryOptimiser(
    const OptionsMap & optimiserMap,
    const OptionsMap * potentialMap = NULL
  ) const;

  build_cell::IStructureGeneratorPtr createStructureGenerator(const OptionsMap & map) const;

  build_cell::StructureBuilderPtr createStructureBuilder(const OptionsMap & map) const;

  build_cell::AtomsGeneratorPtr createAtomsGenerator(
    const OptionsMap & map,
    io::AtomFormatParser & parser
  ) const;

  build_cell::AtomsDescriptionPtr createAtomsDescription(
    const AtomsDataEntry & atomsEntry,
    const io::AtomFormatParser & parser
  ) const;

  build_cell::RandomUnitCellPtr createRandomCellGenerator(const OptionsMap & map) const;

  potential::IPotentialPtr createPotential(const OptionsMap & map) const;
  utility::IStructureComparatorPtr createStructureComparator(const OptionsMap & map) const;
  //UniqueStructureSetPtr                 createStructureSet(const YAML::Node & desc);
  //io::IStructureWriter *                createStructureWriter(const YAML::Node & node);


private:

  typedef ::boost::optional<AtomSpeciesCount> OptionalAtomSpeciesCount;
  typedef ::boost::optional<common::AtomSpeciesId::Value> OptionalSpecies;

  struct StructureContentType : ::boost::noncopyable
  {
    enum Value {UNKNOWN, ATOMS, GROUP};
  };

  StructureContentType::Value getStructureContentType(const AtomsDataEntry & atomsEntry) const;

  common::AtomSpeciesDatabase & myAtomSpeciesDb;
  const GenShapeFactory myShapeFactory;
};

}
}

#endif /* SS_LIB_FACTORY_YAML_H */

