/*
 * StructureYamlGenerator.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "spl/io/StructureYamlGenerator.h"

#include <sstream>

#include <boost/foreach.hpp>

#include <armadillo>

#include "spl/common/Atom.h"
#include "spl/common/AtomSpeciesDatabase.h"
#include "spl/common/Structure.h"
#include "spl/common/UnitCell.h"
#include "spl/factory/SsLibYamlKeywords.h"
#include "spl/io/IoFunctions.h"
#include "spl/utility/IndexingEnums.h"
#include "spl/yaml/Transcode.h"

// DEFINES /////////////////////////////////

// NAMESPACES ////////////////////////////////

namespace spl {
namespace io {

namespace kw = factory::sslib_yaml_keywords;
namespace structure_properties = common::structure_properties;

StructureYamlGenerator::StructureYamlGenerator()
{
  AtomsFormat format;
  const YAML::Node null;
  format.push_back(FormatEntry(kw::STRUCTURE__ATOMS__SPEC, null));
  format.push_back(FormatEntry(kw::STRUCTURE__ATOMS__POS, null));
  myAtomInfoParser.setFormat(format);
}

StructureYamlGenerator::StructureYamlGenerator(
    const AtomYamlFormatParser::AtomsFormat & format) :
    myAtomInfoParser(format)
{
}

YAML::Node
StructureYamlGenerator::generateNode(
    const ::spl::common::Structure & structure) const
{
  using namespace utility::cell_params_enum;

  YAML::Node root;

  // Name
  if(!structure.getName().empty())
    root[kw::STRUCTURE__NAME] = structure.getName();

  // Unit cell
  const common::UnitCell * const cell = structure.getUnitCell();
  if(cell)
    root[kw::STRUCTURE__CELL] = *cell;

  // Atoms
  for(size_t i = 0; i < structure.getNumAtoms(); ++i)
    root[kw::STRUCTURE__ATOMS].push_back(generateNode(structure.getAtom(i)));

  // Properties
  BOOST_FOREACH(const StructureProperty & property, structure_properties::VISIBLE_PROPERTIES)
    addProperty(root[kw::STRUCTURE__PROPERTIES], structure, property);

  return root;
}

common::types::StructurePtr
StructureYamlGenerator::generateStructure(const YAML::Node & node) const
{
  typedef AtomYamlFormatParser::AtomInfo::iterator AtomInfoIterator;

  common::types::StructurePtr structure(new common::Structure());

  if(node[kw::STRUCTURE__NAME])
    structure->setName(node[kw::STRUCTURE__NAME].as< ::std::string>());

  if(node[kw::STRUCTURE__CELL])
    structure->setUnitCell(node[kw::STRUCTURE__CELL].as< common::UnitCell>());

  if(node[kw::STRUCTURE__ATOMS] && node[kw::STRUCTURE__ATOMS].IsSequence())
  {
    AtomYamlFormatParser::AtomInfo atomInfo;
    AtomInfoIterator it;
    common::AtomSpeciesId::Value species;
    ::arma::vec3 pos;
    BOOST_FOREACH(const YAML::Node & atomNode, node[kw::STRUCTURE__ATOMS])
    {
      if(myAtomInfoParser.parse(atomInfo, atomNode))
      {
        const AtomInfoIterator end = atomInfo.end();

        it = atomInfo.find(kw::STRUCTURE__ATOMS__SPEC);
        if(it != end)
        {
          species = it->second.as< ::std::string>();

          common::Atom & atom = structure->newAtom(species);
          it = atomInfo.find(kw::STRUCTURE__ATOMS__POS);
          if(it != end)
          {
            pos = it->second.as< ::arma::vec3>();
            atom.setPosition(pos);
          }
        }

        atomInfo.clear(); // Clear so we can use next time around
      }
      else
      {
        // TODO: Emit error
      }
    }
  }

  if(node[kw::STRUCTURE__PROPERTIES])
    praseProperties(*structure, node[kw::STRUCTURE__PROPERTIES]);

  return structure;
}

YAML::Node
StructureYamlGenerator::generateNode(const ::spl::common::Atom & atom) const
{
  using namespace utility::cart_coords_enum;

  AtomYamlFormatParser::AtomInfo atomInfo;

  BOOST_FOREACH(const FormatEntry & entry, myAtomInfoParser.getFormat())
  {
    if(entry.first == kw::STRUCTURE__ATOMS__SPEC)
    {
      if(!atom.getSpecies().empty())
        atomInfo[kw::STRUCTURE__ATOMS__SPEC] = atom.getSpecies();
    }
    else if(entry.first == kw::STRUCTURE__ATOMS__POS)
      atomInfo[kw::STRUCTURE__ATOMS__POS] = atom.getPosition();
    else if(entry.first == kw::STRUCTURE__ATOMS__RADIUS)
      atomInfo[kw::STRUCTURE__ATOMS__RADIUS] = atom.getRadius();
  }

  return myAtomInfoParser.generateNode(atomInfo);
}

bool
StructureYamlGenerator::addProperty(YAML::Node propertiesNode,
    const common::Structure & structure,
    const StructureProperty & property) const
{
  ::boost::optional< ::std::string> value = structure.getVisibleProperty(
      property);

  if(!value)
    return false;

  propertiesNode[property.getName()] = *value;

  return true;
}

void
StructureYamlGenerator::praseProperties(common::Structure & structure,
    const YAML::Node & propertiesNode) const
{
  common::Structure::VisibleProperty * property;
  for(YAML::const_iterator it = propertiesNode.begin(), end =
      propertiesNode.end(); it != end; ++it)
  {
    property = structure_properties::VISIBLE_PROPERTIES.getProperty(
        it->first.as< ::std::string>());

    if(property)
    {
      structure.setVisibleProperty(*property, it->second.as< ::std::string>());
    }
  }
}

}
}
