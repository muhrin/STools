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

#ifdef SSLIB_USE_YAML

#include <map>

#include <boost/optional.hpp>
#include <boost/ptr_container/ptr_vector.hpp>
#include <boost/utility.hpp>

#include <yaml-cpp/yaml.h>

// Local includes
#include "build_cell/IStructureGenerator.h"
#include "build_cell/StructureDescription.h"
#include "build_cell/Types.h"
#include "factory/SsLibYamlKeywords.h"
#include "factory/YamlCommon.h"
#include "io/IStructureWriter.h"
#include "potential/IGeomOptimiser.h"
#include "potential/IPotential.h"
#include "potential/SimplePairPotential.h"
#include "potential/TpsdGeomOptimiser.h"
#include "utility/IStructureComparator.h"
#include "utility/IStructureSet.h"



// FORWARD DECLARATIONS ////////////////////////////////////

namespace sstbx {
namespace build_cell {
class AtomsDescription;
}
}

namespace sstbx {
namespace factory {

class SsLibFactoryYaml : ::boost::noncopyable
{
public:

  enum ErrorCode
  {
    UNKNOWN,
    BAD_TAG,
    UNRECOGNISED_KEYWORD,
    REQUIRED_KEYWORD_MISSING,
    MALFORMED_VALUE
  };

  build_cell::UnitCellBlueprintPtr               createCellGenerator(const YAML::Node & desc);
  build_cell::StructureDescriptionPtr            createStructureDescription(const YAML::Node & desc);
  potential::IPotential *                        createPotential(const YAML::Node & desc);
  potential::IGeomOptimiser *                    createGeometryOptimiser(const YAML::Node & desc);
  utility::IStructureComparator *                createStructureComparator(const YAML::Node & node);
  utility::IStructureSet *                       createStructureSet(const YAML::Node & desc);
  io::IStructureWriter *                         createStructureWriter(const YAML::Node & node);


private:

  typedef ::boost::optional<double> OptionalDouble;
  typedef ::std::pair<common::AtomSpeciesId::Value, unsigned int> AtomSpeciesCount;
  typedef ::boost::optional<AtomSpeciesCount> OptionalAtomSpeciesCount;

  build_cell::AtomsDescriptionPtr                createAtomsDescription(const YAML::Node & desc, OptionalDouble atomsRadii = OptionalDouble()) const;
  build_cell::AtomConstraintDescription *        createAtomConstraintDescription(const YAML::Node & descNode) const;
  build_cell::StructureConstraintDescription *   createStructureConstraintDescription(const YAML::Node & descNode) const;

  void checkKeyword(const sslib_yaml_keywords::KwTyp & kw, const YAML::Node & node) const;

  OptionalAtomSpeciesCount parseAtomTypeString(const ::std::string & atomSpecString) const;


  ::boost::ptr_vector< ::sstbx::io::IStructureWriter>                          myStructureWriters;
  ::boost::ptr_vector< ::sstbx::potential::IGeomOptimiser>                     myOptimisers;
  ::boost::ptr_vector< ::sstbx::potential::IPotential>                         myPotentials;
  ::boost::ptr_vector< ::sstbx::utility::IStructureComparator >                myStructureComparators;
  ::boost::ptr_vector< ::sstbx::utility::IStructureSet>                        myStructureSets;
  ::boost::ptr_vector< ::sstbx::build_cell::StructureDescription>              myStructureDescriptions;


};

}
}


#endif /* SSLIB_USE_YAML */
#endif /* SS_LIB_FACTORY_YAML_H */

