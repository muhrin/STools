/*
 * SsLibFactoryYaml.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "SSLib.h"

#ifdef SSLIB_USE_YAML

#include "factory/SsLibFactoryYaml.h"

#include <memory>

#include <boost/algorithm/string.hpp>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/tokenizer.hpp>

#include <yaml-cpp/yaml.h>

// Local includes
#include "build_cell/AtomsDescription.h"
#include "build_cell/AtomConstraintDescription.h"
#include "build_cell/AtomsGenerator.h"
#include "build_cell/RandomUnitCellGenerator.h"
#include "build_cell/StructureBuilder.h"
#include "build_cell/StructureConstraintDescription.h"
#include "common/AtomSpeciesDatabase.h"
#include "common/AtomSpeciesId.h"
#include "factory/FactoryError.h"
#include "io/AtomYamlFormatParser.h"
#include "io/ResReaderWriter.h"
#include "potential/Types.h"
#include "utility/IndexingEnums.h"
#include "utility/SortedDistanceComparator.h"



// NAMESPACES ////////////////////////////////

namespace sstbx {
namespace factory {

// namespace aliases
namespace ssbc  = ::sstbx::build_cell;
namespace ssc   = ::sstbx::common;
namespace ssio  = ::sstbx::io;
namespace ssp   = ::sstbx::potential;
namespace ssu   = ::sstbx::utility;

namespace kw = sslib_yaml_keywords;

// Boost Tokenizer stuff
typedef boost::tokenizer<boost::char_separator<char> > Tok;
const boost::char_separator<char> tokSep(" \t");

SsLibFactoryYaml::SsLibFactoryYaml(common::AtomSpeciesDatabase & atomSpeciesDb):
myAtomSpeciesDb(atomSpeciesDb)
{}

common::StructurePtr
SsLibFactoryYaml::createStructure(const YAML::Node & structureNode) const
{
  common::StructurePtr structure(new common::Structure());


  // TODO: Use StructureYamlGenerator here

  return structure;
}

common::UnitCellPtr
SsLibFactoryYaml::createUnitCell(const YAML::Node & cellNode) const
{
  common::UnitCellPtr cell;

  if(cellNode[kw::STRUCTURE__CELL__ABC])
  {
    const YAML::Node & abcNode = cellNode[kw::STRUCTURE__CELL__ABC];


  }

  return cell;
}

ssbc::RandomUnitCellPtr
SsLibFactoryYaml::createRandomCellGenerator(const YAML::Node & node) const
{
  using namespace utility::cell_params_enum;

  ssbc::RandomUnitCellPtr cell(new ssbc::RandomUnitCellGenerator());

  if(node[kw::RANDOM_CELL__ABC])
  {
    const YAML::Node & paramsNode = node[kw::RANDOM_CELL__ABC];

    if(paramsNode.IsSequence() && paramsNode.size() == 6)
    {
      double params[6];
      for(size_t i = A; i <= GAMMA; ++i)
      {
        try
        {
          params[i] = paramsNode[i].as<double>();
        }
        catch(YAML::TypedBadConversion<double> e)
        {
          BOOST_THROW_EXCEPTION(FactoryError() <<
            ErrorType(MALFORMED_VALUE) <<
            NodeName(kw::RANDOM_CELL__ABC) <<
            ProblemValue(paramsNode[i].as< ::std::string>()));
        }
        cell->setMin(i, params[i]);
        cell->setMax(i, params[i]);
      }
    }
    else
    {
      BOOST_THROW_EXCEPTION(FactoryError() <<
        ErrorType(SEQUENCE_LENGTH_INVALID) <<
        NodeName(kw::RANDOM_CELL__ABC));
    }
  }

  OptionalDouble dValue;

  dValue = getNodeChildValueAs<double>(node, kw::RANDOM_CELL__VOL);
  if(dValue)
  {
    cell->setTargetVolume(*dValue);
  }

  if(node[kw::RANDOM_CELL__ANGLES])
  {
    const OptionalMinMax<double>::Type minMax = getMinMax<double>(node[kw::RANDOM_CELL__ANGLES]);
    if(minMax.first)
    {
      cell->setMinAngles(*minMax.first);
    }
    if(minMax.second)
    {
      cell->setMaxAngles(*minMax.second);
    }
  }

  if(node[kw::RANDOM_CELL__LENGTHS])
  {
    const YAML::Node & lengths = node[kw::RANDOM_CELL__LENGTHS];
    const OptionalMinMax<double>::Type minMax = getMinMax<double>(lengths);
    if(minMax.first)
    {
      cell->setMinLengths(*minMax.first);
    }
    if(minMax.second)
    {
      cell->setMaxLengths(*minMax.second);
    }
    dValue = getNodeChildValueAs<double>(lengths, kw::RANDOM_CELL__LENGTHS__MAX_RATIO);
    if(dValue)
    {
      cell->setMaxLengthRatio(dValue);
    }
  }

  return cell;
}

ssbc::IStructureGeneratorPtr
SsLibFactoryYaml::createStructureGenerator(const YAML::Node & node) const
{
  //// Make sure we have a structure description node
  //if(node.Scalar() != kw::STR_DESC)
  //{
  //  throw FactoryError() << ErrorType(BAD_TAG) << NodeName(node.Scalar());
  //}

  ssbc::IStructureGeneratorPtr generator;

  if(node[kw::RANDOM_STRUCTURE])
  {
    generator = createStructureBuilder(node[kw::RANDOM_STRUCTURE]);
  }

  // Assign the pointer so the caller gets the object
  return generator;
}

SsLibFactoryYaml::PotentialPtr
SsLibFactoryYaml::createPotential(const YAML::Node & node) const
{
  PotentialPtr pot;

  const OptionalString potType = getNodeChildValueAs< ::std::string>(node, kw::POTENTIAL__TYPE);
  if(!potType) // If no type then we can't construct the potential
    return pot;

  if(*potType == kw::POTENTIAL__TYPE___LENNARD_JONES)
  {
    // Get the species list
    AtomSpeciesIdVector speciesVec(myAtomSpeciesDb);
    ::boost::optional<AtomSpeciesIdVector> species(speciesVec);
    if(!getChildNodeValue(species, node, kw::POTENTIAL__LENNARD_JONES__SPECIES))
      return pot;

    // Build up the potential parameters one by one
    OptionalArmaTriangularMat epsilon;
    if(!getChildNodeValue(epsilon, node, kw::POTENTIAL__LENNARD_JONES__EPS))
      return pot;

    OptionalArmaTriangularMat sigma;
    if(!getChildNodeValue(sigma, node, kw::POTENTIAL__LENNARD_JONES__SIG))
      return pot;

    OptionalArmaTriangularMat beta;
    if(!getChildNodeValue(beta, node, kw::POTENTIAL__LENNARD_JONES__BETA))
      return pot;

    OptionalArmaVec pow;
    if(!getChildNodeValue(pow, node, kw::POTENTIAL__LENNARD_JONES__POW))
      return pot;

    OptionalDouble cutoff;
    if(!getChildNodeValue(cutoff, node, kw::POTENTIAL__LENNARD_JONES__CUT))
      return pot;

    ::boost::optional<ssp::SimplePairPotential::CombiningRule> comb;
    if(!getChildNodeValue(comb, node, kw::POTENTIAL__LENNARD_JONES__COMB))
      return pot;

    const size_t numSpecies = (*epsilon)->n_rows;

    if((numSpecies != (*sigma)->n_rows) || (numSpecies != (*beta)->n_rows))
      return pot;

    pot.reset(new ssp::SimplePairPotential(
      myAtomSpeciesDb,
      **species,
      **epsilon,
      **sigma,
      *cutoff,
      **beta,
      (*pow)(0),
      (*pow)(1),
      *comb
    ));
  }

  return pot;
}



SsLibFactoryYaml::GeomOptimiserPtr
SsLibFactoryYaml::createGeometryOptimiser(const YAML::Node & node) const
{
  GeomOptimiserPtr opt;

  OptionalString type;
  if(!getChildNodeValue(type, node, kw::OPTIMISER__TYPE))
    return opt;

  if(*type == kw::OPTIMISER__TYPE___TPSD)
  {
    // Have to have a potential with this optimiser
    if(!node[kw::OPTIMISER__POTENTIAL])
      return opt;
    potential::IPotentialPtr potential = createPotential(node[kw::OPTIMISER__POTENTIAL]);
    if(!potential.get())
      return opt;

    const ::boost::optional<double> tolerance(
      getNodeChildValueAs<double>(node, kw::OPTIMISER__TPSD__TOL)
    );

    if(tolerance)
      opt.reset(new ssp::TpsdGeomOptimiser(potential, *tolerance));
    else
      opt.reset(new ssp::TpsdGeomOptimiser(potential));
  }

  return opt;
}

::sstbx::utility::IStructureComparator *
SsLibFactoryYaml::createStructureComparator(const YAML::Node & node)
{
  //// Make sure we have a structure set node
  //if(node.Scalar() != kw::STR_COMPARATOR)
  //{
  //  throw FactoryError() << ErrorType(BAD_TAG) << NodeName(node.Scalar());
  //}

  checkKeyword(kw::TYPE, node);

  const ::std::string type = node[kw::TYPE].as< ::std::string>();

  ssu::IStructureComparator * comparator = NULL;

  if(type == kw::STR_COMPARATOR__TYPE___SORTED_DIST)
  {
    comparator = new ssu::SortedDistanceComparator();
    myStructureComparators.push_back(comparator);
  }
  else
  {
    throw FactoryError() << ErrorType(UNRECOGNISED_KEYWORD);
  }

  return comparator;
}

SsLibFactoryYaml::UniqueStructureSetPtr
SsLibFactoryYaml::createStructureSet(const YAML::Node & node)
{
  //// Make sure we have a structure set node
  //if(node.Scalar() != kw::STR_SET)
  //{
  //  throw FactoryError() << ErrorType(BAD_TAG) << NodeName(node.Scalar());
  //}

  // First try to create the comparator
  checkKeyword(kw::STR_COMPARATOR, node);

  ssu::IStructureComparator * const comparator = createStructureComparator(node[kw::STR_COMPARATOR]);

  UniqueStructureSetPtr strSet;
  if(comparator)
  {
    strSet.reset(new ssu::UniqueStructureSet<>(*comparator));
  }
  
  return strSet;
}

::sstbx::io::IStructureWriter *
SsLibFactoryYaml::createStructureWriter(const YAML::Node & node)
{
  //// Make sure we have a structure writer tag
  //if(node.Scalar() != kw::STR_SET)
  //{
  //  throw FactoryError() << ErrorType(BAD_TAG) << NodeName(node.Scalar());
  //}

  // Check we have the required keywords
  checkKeyword(kw::TYPE, node);

  const ::std::string type = node[kw::TYPE].as< ::std::string>();

  ssio::IStructureWriter * writer = NULL;
  if(type == kw::STR_WRITER__TYPE___RES)
  {
    writer = new ssio::ResReaderWriter();
  }

  if(writer)
    myStructureWriters.push_back(writer);

  return writer;
}

void SsLibFactoryYaml::checkKeyword(
  const kw::KwTyp & kw,
  const YAML::Node & node) const
{
  if(!node[kw])
    throw FactoryError() << ErrorType(REQUIRED_KEYWORD_MISSING);
}


ssbc::StructureConstraintDescription *
SsLibFactoryYaml::createStructureConstraintDescription(const YAML::Node & descNode) const
{
  // TODO: No structure constraint
  return NULL;
}

SsLibFactoryYaml::OptionalNode
SsLibFactoryYaml::getChildNode(const YAML::Node & parent, const ::std::string & childNodeName) const
{
  OptionalNode childNode;

  if(parent[childNodeName])
  {
    childNode.reset(parent[childNodeName]);
  }

  return childNode;
}

SsLibFactoryYaml::OptionalAtomSpeciesCount
SsLibFactoryYaml::parseAtomTypeString(const ::std::string & atomSpecString) const
{
  OptionalAtomSpeciesCount atomSpeciesCount;

  const Tok tok(atomSpecString, tokSep);

  Tok::const_iterator it = tok.begin();

  AtomSpeciesCount type;
  type.second = 1;
  bool successful = false;
  if(it != tok.end())
  {
    try
    {
      type.second = ::boost::lexical_cast<unsigned int>(*it);
      ++it;
    }
    catch(::boost::bad_lexical_cast)
    {}

    if(it != tok.end())
    {
      type.first = myAtomSpeciesDb.getIdFromSymbol(*it);
      if(type.first != common::AtomSpeciesId::DUMMY)
        successful = true;
    }
  }

  if(successful)
    atomSpeciesCount.reset(type);

  return atomSpeciesCount;
}

SsLibFactoryYaml::StructureContentType::Value
SsLibFactoryYaml::getStructureContentType(const YAML::Node & node) const
{
  if(node[kw::STRUCTURE__ATOMS__SPEC] || node.IsSequence())
    return StructureContentType::ATOMS;
  if(node[kw::RANDOM_STRUCTURE__ATOMS__GROUP])
    return StructureContentType::GROUP;

  return StructureContentType::UNKNOWN;
}

build_cell::AtomsDescriptionPtr
SsLibFactoryYaml::createAtomsDescription(
  const YAML::Node & descNode,
  const io::AtomYamlFormatParser & parser,
  const AtomsDefaults & defaults) const
{
  std::string sValue;

  io::AtomYamlFormatParser::AtomInfo atomsInfo;
  if(!parser.parse(atomsInfo, descNode))
  {
    // TODO: Throw exception
  }
  io::AtomYamlFormatParser::AtomInfo::const_iterator it, end = atomsInfo.end();

  unsigned int nAtoms = defaults.count;
  ssc::AtomSpeciesId::Value species;
  if(defaults.species)
    species = *defaults.species;


  it = atomsInfo.find(kw::STRUCTURE__ATOMS__SPEC);
  if(it != end)
  {
    sValue = it->second.as< ::std::string>();
    OptionalAtomSpeciesCount atomSpeciesCount = parseAtomTypeString(sValue);
    if(atomSpeciesCount)
    {
      species = atomSpeciesCount->first;
      nAtoms = atomSpeciesCount->second;
    }
  }

  build_cell::AtomsDescriptionPtr atomsDescription(new build_cell::AtomsDescription(species, nAtoms));

  // If there is a default atoms radii first set that and then try to see if there
  // is a specific one for this atom
  if(defaults.radius)
    atomsDescription->setRadius(*defaults.radius);

  it = atomsInfo.find(kw::RANDOM_STRUCTURE__ATOMS__RADIUS);
  if(it != end)
  {
    atomsDescription->setRadius(it->second.as<double>());
  }


  return atomsDescription;
}

build_cell::StructureBuilderPtr
SsLibFactoryYaml::createStructureBuilder(const YAML::Node & node) const
{
  build_cell::StructureBuilderPtr builder(new ssbc::StructureBuilder());

  const OptionalDouble atomsRadii = getNodeChildValueAs<double>(node, kw::RANDOM_STRUCTURE__ATOMS_RADII);

  io::AtomYamlFormatParser atomsFormatParser;
  if(node[kw::STRUCTURE__ATOMS_FORMAT])
    atomsFormatParser.updateFormat(node[kw::STRUCTURE__ATOMS_FORMAT]);

  // Generators 
  if(node[kw::RANDOM_STRUCTURE__ATOMS])
  {
    // Try creating the default atoms generator
    {
      build_cell::AtomsGeneratorPtr atomsGenerator = createAtomsGenerator(node, atomsFormatParser);
      if(atomsGenerator.get())
        builder->addGenerator(atomsGenerator);
    }

    const YAML::Node & atomsNode = node[kw::RANDOM_STRUCTURE__ATOMS];

    // Now look for other generators
    BOOST_FOREACH(const YAML::Node & atomNode, atomsNode)
    {
      if(getStructureContentType(atomNode) == StructureContentType::GROUP)
      {
        build_cell::AtomsGeneratorPtr atomsGenerator = createAtomsGenerator(atomNode, atomsFormatParser);
        if(atomsGenerator.get())
          builder->addGenerator(atomsGenerator);
      }
    }
  }

  // Unit cell
  if(node[kw::RANDOM_CELL])
  {
    build_cell::IUnitCellGeneratorPtr ucGen(createRandomCellGenerator(node[kw::RANDOM_CELL]));
    builder->setUnitCellGenerator(ucGen);
  }

  return builder;
}

build_cell::AtomsGeneratorPtr
SsLibFactoryYaml::createAtomsGenerator(
  const YAML::Node & node,
  const io::AtomYamlFormatParser & atomsFormatParser,
  AtomsDefaults defaults
) const
{
  build_cell::AtomsGeneratorPtr atomsGenerator(new build_cell::AtomsGenerator());

  if(node[kw::RANDOM_STRUCTURE__ATOMS_RADII])
    defaults.radius = node[kw::RANDOM_STRUCTURE__ATOMS_RADII].as<double>();

  ///////////
  // Atoms
  if(node[kw::RANDOM_STRUCTURE__ATOMS])
  {
    const YAML::Node & atomsNode = node[kw::RANDOM_STRUCTURE__ATOMS];

    if(atomsNode.IsSequence())
    {
      BOOST_FOREACH(const YAML::Node & atomNode, atomsNode)
      {
        if(getStructureContentType(atomNode) == StructureContentType::ATOMS)
          atomsGenerator->addAtoms(*createAtomsDescription(atomNode, atomsFormatParser, defaults));
      }
    }
    else
      BOOST_THROW_EXCEPTION(FactoryError() << ErrorType(MALFORMED_VALUE));
  }

  if(node[kw::RANDOM_STRUCTURE__ATOMS__GEN_SPHERE])
  {
    try
    {
      atomsGenerator->setGenerationSphere(
        node[kw::RANDOM_STRUCTURE__ATOMS__GEN_SPHERE].as<build_cell::Sphere>()
      );
    }
    catch(const YAML::Exception & /*exception*/)
    {
      BOOST_THROW_EXCEPTION(FactoryError() << ErrorType(MALFORMED_VALUE));
    }
  }

  return atomsGenerator;
}

}
}


#endif /* SSLIB_USE_YAML */
