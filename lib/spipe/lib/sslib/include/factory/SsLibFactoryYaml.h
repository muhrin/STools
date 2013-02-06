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

#include <boost/noncopyable.hpp>
#include <boost/optional.hpp>
#include <boost/ptr_container/ptr_vector.hpp>
#include <boost/utility.hpp>

#include <yaml-cpp/yaml.h>

// Local includes
#include "OptionalTypes.h"
#include "build_cell/IStructureGenerator.h"
#include "build_cell/StructureConstraintDescription.h"
#include "build_cell/Types.h"
#include "factory/FactoryError.h"
#include "factory/SsLibYamlKeywords.h"
#include "io/IStructureWriter.h"
#include "io/YamlTranscode.h"
#include "potential/IGeomOptimiser.h"
#include "potential/IPotential.h"
#include "potential/SimplePairPotential.h"
#include "potential/TpsdGeomOptimiser.h"
#include "utility/IStructureComparator.h"
#include "utility/UniqueStructureSet.h"



// FORWARD DECLARATIONS ////////////////////////////////////

namespace sstbx {
namespace build_cell {
class AtomsDescription;
}
namespace common {
class AtomSpeciesDatabase;
}
namespace io {
class AtomYamlFormatParser;
}

namespace factory {

class SsLibFactoryYaml : ::boost::noncopyable
{
public:
  typedef UniquePtr<potential::IGeomOptimiser>::Type GeomOptimiserPtr;
  typedef UniquePtr<potential::IPotential>::Type PotentialPtr;
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

  common::StructurePtr                  createStructure(const YAML::Node & structureNode) const;
  common::UnitCellPtr                   createUnitCell(const YAML::Node & cellNode) const;
  build_cell::RandomUnitCellPtr         createRandomCellGenerator(const YAML::Node & desc) const;
  build_cell::IStructureGeneratorPtr    createStructureGenerator(const YAML::Node & desc) const;
  GeomOptimiserPtr                      createGeometryOptimiser(const YAML::Node & desc) const;
  PotentialPtr                          createPotential(const YAML::Node & node) const;
  utility::IStructureComparator *       createStructureComparator(const YAML::Node & node);
  UniqueStructureSetPtr                 createStructureSet(const YAML::Node & desc);
  io::IStructureWriter *                createStructureWriter(const YAML::Node & node);


private:

  typedef ::std::pair<common::AtomSpeciesId::Value, unsigned int> AtomSpeciesCount;
  typedef ::boost::optional<AtomSpeciesCount> OptionalAtomSpeciesCount;
  typedef ::boost::optional<YAML::Node> OptionalNode;
  typedef ::boost::optional<const YAML::Node> OptionalConstNode;
  typedef ::boost::optional<common::AtomSpeciesId::Value> OptionalSpecies;
  typedef ::boost::optional<factory::ArmaTriangularMat> OptionalArmaTriangularMat;

  template <typename T>
  struct OptionalMinMax : ::boost::noncopyable
  {
    typedef ::std::pair< ::boost::optional<T>, ::boost::optional<T> > Type;
  private:
    OptionalMinMax() {}
  };

  struct MinMaxRequire : ::boost::noncopyable
  {
    enum Value
    {
      NEITHER = 0x00,   // 000
      MIN     = 0x01,   // 001
      MAX     = 0x02,   // 010
      BOTH    = 0x03,   // 011
    };
  };

  struct StructureContentType : ::boost::noncopyable
  {
    enum Value {UNKNOWN, ATOMS, GROUP};
  };

  struct AtomsDefaults
  {
    AtomsDefaults():count(1) {}

    unsigned int count;
    OptionalDouble radius;
    OptionalSpecies species;
  };


  build_cell::StructureConstraintDescription *   createStructureConstraintDescription(const YAML::Node & descNode) const;

  void checkKeyword(const sslib_yaml_keywords::KwTyp & kw, const YAML::Node & node) const;

  OptionalNode getChildNode(const YAML::Node & parent, const ::std::string & childNodeName) const;

  OptionalAtomSpeciesCount parseAtomTypeString(const ::std::string & atomSpecString) const;

  template <typename T>
  ::boost::optional<T> getNodeChildValueAs(const YAML::Node & node, const ::std::string & childName) const;

  template <typename T>
  bool getChildNodeValue(
    ::boost::optional<T> & value,
    const YAML::Node & node,
    const ::std::string & childName
  ) const;

  template <typename T>
  typename OptionalMinMax<T>::Type getMinMax(const YAML::Node & parentNode) const;

  StructureContentType::Value getStructureContentType(const YAML::Node & node) const;
  build_cell::AtomsDescriptionPtr createAtomsDescription(
    const YAML::Node & descNode,
    const io::AtomYamlFormatParser & parser,
    const AtomsDefaults & detauls = AtomsDefaults()
  ) const;
  build_cell::StructureBuilderPtr createStructureBuilder(const YAML::Node & desc) const;
  build_cell::AtomsGeneratorPtr createAtomsGenerator(
    const YAML::Node & desc,
    const io::AtomYamlFormatParser & parser,
    AtomsDefaults detauls = AtomsDefaults()
  ) const;

  common::AtomSpeciesDatabase & myAtomSpeciesDb;

  ::boost::ptr_vector< ::sstbx::io::IStructureWriter>                          myStructureWriters;
  ::boost::ptr_vector< ::sstbx::utility::IStructureComparator >                myStructureComparators;

};

// TYPEDEFS /////////////////
typedef ::boost::error_info<struct TagErrorType, SsLibFactoryYaml::ErrorCode>    ErrorType;
typedef ::boost::error_info<struct TagNodeName, ::std::string>                   NodeName;
typedef ::boost::error_info<struct TagValue, ::std::string>                      ProblemValue;

template <typename T>
::boost::optional<T>
SsLibFactoryYaml::getNodeChildValueAs(
  const YAML::Node & node,
  const ::std::string & childName
) const
{
  ::boost::optional<T> childValue;
  if(node[childName])
  {
    const YAML::Node & childNode = node[childName];
    if(childNode.IsScalar())
    {
      try { childValue.reset(childNode.as<T>()); }
      catch(const YAML::TypedBadConversion<T> & /*e*/)  {}
    }
  }
  return childValue;
}

template <typename T>
bool SsLibFactoryYaml::getChildNodeValue(
  ::boost::optional<T> & value,
  const YAML::Node & node,
  const ::std::string & childName
) const
{
  bool valueFound = false;
  if(node[childName])
  {
    const YAML::Node & childNode = node[childName];
    if(childNode.IsScalar())
    {
      try
      {
        value.reset(childNode.as<T>());
        valueFound = true;
      }
      catch(const YAML::TypedBadConversion<T> & /*e*/)  {}
    }
  }
  return valueFound;
}

template <typename T>
typename SsLibFactoryYaml::OptionalMinMax<T>::Type
SsLibFactoryYaml::getMinMax(const YAML::Node & parentNode) const
{
  typename OptionalMinMax<T>::Type minMax;

  minMax.first  = getNodeChildValueAs<T>(parentNode, sslib_yaml_keywords::MIN);
  minMax.second = getNodeChildValueAs<T>(parentNode, sslib_yaml_keywords::MAX);

  return minMax;
}

}
}


#endif /* SSLIB_USE_YAML */
#endif /* SS_LIB_FACTORY_YAML_H */

