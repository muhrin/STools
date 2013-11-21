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
#include "spl/SSLib.h"

#include <map>

#include <boost/noncopyable.hpp>
#include <boost/optional.hpp>
#include <boost/ptr_container/ptr_vector.hpp>
#include <boost/utility.hpp>

#include "spl/OptionalTypes.h"
#include "spl/build_cell/BuildCellFwd.h"
#include "spl/build_cell/StructureBuilder.h"
#include "spl/build_cell/StructureGenerator.h"
#include "spl/factory/FactoryFwd.h"
#include "spl/factory/GenShapeFactory.h"
#include "spl/io/AtomFormatParser.h"
#include "spl/io/IStructureWriter.h"
#include "spl/potential/Types.h"
#include "spl/utility/HeterogeneousMapKey.h"
#include "spl/utility/IStructureComparator.h"
#include "spl/utility/UniqueStructureSet.h"

// FORWARD DECLARATIONS ////////////////////////////////////

namespace spl {
namespace common {
class AtomSpeciesDatabase;
}
namespace potential {
class OptimisationSettings;
}

namespace factory {

class SsLibFactoryYaml : ::boost::noncopyable
{
  typedef utility::HeterogeneousMap OptionsMap;
public:
  typedef UniquePtr< potential::IGeomOptimiser>::Type GeomOptimiserPtr;
  typedef UniquePtr< utility::UniqueStructureSet< > >::Type UniqueStructureSetPtr;

  SsLibFactoryYaml(common::AtomSpeciesDatabase & atomSpeciesDb);

  GeomOptimiserPtr
  createGeometryOptimiser(const OptionsMap & optimiserOptions) const;
  potential::OptimisationSettings
  createOptimisationSettings(const OptionsMap & options) const;
  build_cell::IStructureGeneratorPtr
  createStructureGenerator(const OptionsMap & map) const;
  build_cell::StructureBuilderPtr
  createStructureBuilder(const OptionsMap & map) const;
  build_cell::AtomsDescriptionPtr
  createAtomsDescription(const AtomsDataEntry & atomsEntry,
      const io::AtomFormatParser & parser) const;
  build_cell::AtomsGroupPtr
  createAtomsGroup(const OptionsMap & map, io::AtomFormatParser & parser) const;
  build_cell::RandomUnitCellPtr
  createRandomCellGenerator(const OptionsMap & map) const;
  potential::IPotentialPtr
  createPotential(const OptionsMap & map) const;
  utility::IStructureComparatorPtr
  createStructureComparator(const OptionsMap & map) const;

private:

  typedef ::boost::optional< AtomSpeciesCount> OptionalAtomSpeciesCount;
  typedef ::boost::optional< common::AtomSpeciesId::Value> OptionalSpecies;

  struct StructureContentType : ::boost::noncopyable
  {
    enum Value
    {
      UNKNOWN, ATOMS, GROUP
    };
  };

  template <typename T>
  ::boost::optional<T> toOptional(const T * ptr) const;

  StructureContentType::Value
  getStructureContentType(const AtomsDataEntry & atomsEntry) const;

  template< typename T>
    const T *
    find(const utility::Key< T> & key, const OptionsMap & options,
        const OptionsMap * globalOptions) const;

  common::AtomSpeciesDatabase & myAtomSpeciesDb;
  const GenShapeFactory myShapeFactory;
};

template <typename T>
::boost::optional<T> SsLibFactoryYaml::toOptional(const T * ptr) const
 {
   ::boost::optional<T> ret;
   if(ptr)
     ret.reset(*ptr);
   return ret;
 }

template< typename T>
  const T *
  SsLibFactoryYaml::find(const utility::Key< T> & key,
      const OptionsMap & options, const OptionsMap * globalOptions) const
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

