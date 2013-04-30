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
namespace potential {
class IControllableOptimiser;
class LandscapeExplorerOptimiser;
}

namespace factory {

class SsLibFactoryYaml : ::boost::noncopyable
{
  typedef utility::HeterogeneousMap OptionsMap;
public:
  typedef UniquePtr<potential::IGeomOptimiser>::Type GeomOptimiserPtr;
  typedef UniquePtr<potential::IControllableOptimiser>::Type ControllableOptimiserPtr;
  typedef UniquePtr<potential::LandscapeExplorerOptimiser>::Type ExplorerOptimiserPtr;
  typedef UniquePtr<utility::UniqueStructureSet<> >::Type UniqueStructureSetPtr;

  SsLibFactoryYaml(common::AtomSpeciesDatabase & atomSpeciesDb);
  
  GeomOptimiserPtr createGeometryOptimiser(
    const OptionsMap & optimiserOptions,
    const OptionsMap * potentialOptions = NULL,
    const OptionsMap * globalOptions = NULL
  ) const;
  ControllableOptimiserPtr createControllableOptimiser(
    const OptionsMap & optimiserOptions,
    const OptionsMap * potentialOptions = NULL,
    const OptionsMap * globalOptions = NULL
  ) const;
  ExplorerOptimiserPtr createLandscapeExplorerOptimiser(
    const OptionsMap & explorerOptions,
    const OptionsMap & optimiserOptions,
    const OptionsMap * potentialOptions = NULL,
    const OptionsMap * globalOptions = NULL
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

private:

  typedef ::boost::optional<AtomSpeciesCount> OptionalAtomSpeciesCount;
  typedef ::boost::optional<common::AtomSpeciesId::Value> OptionalSpecies;

  struct StructureContentType : ::boost::noncopyable
  {
    enum Value {UNKNOWN, ATOMS, GROUP};
  };

  StructureContentType::Value getStructureContentType(const AtomsDataEntry & atomsEntry) const;

	template<typename T>
	const T * find(
    const utility::Key<T> & key,
    const OptionsMap & options,
    const OptionsMap * globalOptions
  ) const;

  common::AtomSpeciesDatabase & myAtomSpeciesDb;
  const GenShapeFactory myShapeFactory;
};

template <typename T>
const T * SsLibFactoryYaml::find(
  const utility::Key<T> & key,
  const OptionsMap & options,
  const OptionsMap * globalOptions
) const
{
  const T * value = options.find(key);
  // If the value isn't present and we have global options then
  // try to find it there
  if(!value && globalOptions)
    value = globalOptions->find(key);

  return value;
}

}
}

#endif /* SS_LIB_FACTORY_YAML_H */

