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

#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/tokenizer.hpp>

#include <yaml-cpp/yaml.h>

// Local includes
#include "build_cell/AtomsDescription.h"
#include "build_cell/AtomConstraintDescription.h"
#include "build_cell/DefaultCrystalGenerator.h"
#include "build_cell/RandomUnitCell.h"
#include "build_cell/StructureConstraintDescription.h"
#include "common/AtomSpeciesDatabase.h"
#include "common/AtomSpeciesId.h"
#include "factory/FactoryError.h"
#include "io/ResReaderWriter.h"
#include "utility/CellParamsEnum.h"
#include "utility/SortedDistanceComparator.h"
#include "utility/UniqueStructureSet.h"



// NAMESPACES ////////////////////////////////

namespace sstbx {
namespace factory {

// TYPEDEFS /////////////////
typedef ::boost::error_info<struct TagErrorType, SsLibFactoryYaml::ErrorCode>    ErrorType;
typedef ::boost::error_info<struct TagNodeName, ::std::string>                   NodeName;
typedef ::boost::error_info<struct TagKeyword, ::std::string>                    Keyword;

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

ssbc::UnitCellBlueprintPtr
SsLibFactoryYaml::createCellGenerator(const YAML::Node & node)
{
  using namespace utility::cell_params_enum;
  //// Make sure we have a cell description node
  //if(node.Scalar() != kw::CELL_DESC)
  //{
  //  throw FactoryError() << ErrorType(BAD_TAG) << NodeName(node.Scalar()) << Keyword(kw::CELL_DESC);
  //}

  ssbc::RandomUnitCellPtr cell(new ssbc::RandomUnitCell());

  double dValue;

  if(node[kw::CELL_RANDOM__PARAMS])
  {
    const YAML::Node & paramsNode = node[kw::CELL_RANDOM__PARAMS];

    if(paramsNode.IsSequence() && paramsNode.size() == 6)
    {
      double params[6];
      for(size_t i = A; i <= GAMMA; ++i)
      {
        params[i] = paramsNode[i].as<double>();
        cell->setMin(i, params[i]);
        cell->setMax(i, params[i]);
      }
    }
    else
    {
      throw FactoryError() << ErrorType(MALFORMED_VALUE);
    }
  }

  if(node[kw::CELL_RANDOM__VOL])
  {
    dValue = node[kw::CELL_RANDOM__VOL].as<double>();
    cell->setTargetVolume(dValue);
  }

  if(node[kw::CELL_RANDOM__MIN_ANGLE])
  {
    dValue = node[kw::CELL_RANDOM__MIN_ANGLE].as<double>();
    cell->setMinAngles(dValue);
  }

  if(node[kw::CELL_RANDOM__MAX_ANGLE])
  {
    dValue = node[kw::CELL_RANDOM__MAX_ANGLE].as<double>();
    cell->setMaxAngles(dValue);
  }

  return (ssbc::UnitCellBlueprintPtr)cell;
}

ssbc::StructureDescriptionPtr
SsLibFactoryYaml::createStructureDescription(const YAML::Node & node)
{
  //// Make sure we have a structure description node
  //if(node.Scalar() != kw::STR_DESC)
  //{
  //  throw FactoryError() << ErrorType(BAD_TAG) << NodeName(node.Scalar());
  //}

  ssbc::StructureDescriptionPtr strDescription(new ssbc::StructureDescription());

  ::boost::optional<double> atomsRadii;
  if(node[kw::STRUCTURE__ATOMS_RADII])
  {
    const YAML::Node & atomsRadiiNode = node[kw::STRUCTURE__ATOMS_RADII];

    if(atomsRadiiNode.IsScalar())
    {
      atomsRadii.reset(atomsRadiiNode.as<double>());
    }
  }

  // Atoms //
  if(node[kw::STR_DESC__ATOMS])
  {
    const YAML::Node & atomsNode = node[kw::STR_DESC__ATOMS];

    if(atomsNode.IsSequence())
    {
      BOOST_FOREACH(const YAML::Node & atomNode, atomsNode)
      {
        strDescription->addChild(createAtomsDescription(atomNode, atomsRadii));
      }
    }
    else
    {
      throw FactoryError() << ErrorType(MALFORMED_VALUE);
    }
  }

  // Unit cell
  if(node[kw::CELL])
  {
    strDescription->setUnitCell((ssbc::ConstUnitCellBlueprintPtr)createCellGenerator(node[kw::CELL]));
  }

  // Assign the pointer so the caller gets the object
  return strDescription;
}

ssp::IPotential * SsLibFactoryYaml::createPotential(const YAML::Node & node)
{
  //// Make sure we have a potential node
  //if(node.Scalar() != kw::POTENTIAL)
  //{
  //  throw FactoryError() << ErrorType(BAD_TAG) << NodeName(node.Scalar());
  //}

  ssp::IPotential * pot = NULL;

  // First check that we have the keywords that we require in this node
  checkKeyword(kw::TYPE, node);

  const ::std::string potType = node[kw::TYPE].as< ::std::string>();

  if(potType == kw::POTENTIAL__TYPE___PAIR_POT)
  {
    // Check all the required keywords are there for the pair potential
    checkKeyword(kw::POTENTIAL__PAIR_POT__E, node);
    checkKeyword(kw::POTENTIAL__PAIR_POT__S, node);
    checkKeyword(kw::POTENTIAL__PAIR_POT__CUT, node);
    checkKeyword(kw::POTENTIAL__PAIR_POT__B, node);
    checkKeyword(kw::POTENTIAL__PAIR_POT__POW, node);
    checkKeyword(kw::POTENTIAL__PAIR_POT__COMB, node);

    // Let's build it up one by one
    const ArmaTriangularMat epsilon  = node[kw::POTENTIAL__PAIR_POT__E].as<ArmaTriangularMat>();
    const ArmaTriangularMat sigma    = node[kw::POTENTIAL__PAIR_POT__S].as<ArmaTriangularMat>();
    double                  cutoff   = node[kw::POTENTIAL__PAIR_POT__CUT].as<double>();
    const ArmaTriangularMat beta     = node[kw::POTENTIAL__PAIR_POT__B].as<ArmaTriangularMat>();
    const ::arma::vec       pow      = node[kw::POTENTIAL__PAIR_POT__POW].as< ::arma::vec>();
    const ssp::SimplePairPotential::CombiningRule comb =
      node[kw::POTENTIAL__PAIR_POT__COMB].as<ssp::SimplePairPotential::CombiningRule>();

    const size_t numSpecies = epsilon.mat.n_rows;

    if((numSpecies != sigma.mat.n_rows) || (numSpecies != beta.mat.n_rows))
    {
      throw FactoryError() << ErrorType(MALFORMED_VALUE);
    }

    // TODO: HACK!! Fix this to read in the real values
    const ::std::vector<ssc::AtomSpeciesId> species;

    pot = new ssp::SimplePairPotential(
      numSpecies,
      species,
      epsilon.mat,
      sigma.mat,
      cutoff,
      beta.mat,
      pow(0),
      pow(1),
      comb
    );

    myPotentials.push_back(pot);
  }
  else
  {
    throw FactoryError() << ErrorType(UNRECOGNISED_KEYWORD) << Keyword(potType);
  }



  return pot;

}



ssp::IGeomOptimiser * SsLibFactoryYaml::createGeometryOptimiser(const YAML::Node & node)
{
  checkKeyword(kw::TYPE, node);

  ssp::IGeomOptimiser * opt = NULL;

  const ::std::string type = node[kw::TYPE].as< ::std::string>();

  if(type == kw::OPTIMISER__TYPE___TPSD)
  {
    checkKeyword(kw::POTENTIAL, node);

    ssp::IPotential * pp = createPotential(node[kw::POTENTIAL]);

    if(pp)
    {
      opt = new ssp::TpsdGeomOptimiser(*pp);
    }
  }

  myOptimisers.push_back(opt);

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

::sstbx::utility::IStructureSet * SsLibFactoryYaml::createStructureSet(const YAML::Node & node)
{
  //// Make sure we have a structure set node
  //if(node.Scalar() != kw::STR_SET)
  //{
  //  throw FactoryError() << ErrorType(BAD_TAG) << NodeName(node.Scalar());
  //}

  // First try to create the comparator
  checkKeyword(kw::STR_COMPARATOR, node);

  ssu::IStructureComparator * const comparator = createStructureComparator(node[kw::STR_COMPARATOR]);

  ssu::IStructureSet * strSet = NULL;
  if(comparator)
  {
    strSet = new ssu::UniqueStructureSet(*comparator);
    myStructureSets.push_back(strSet);
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

build_cell::AtomsDescriptionPtr
SsLibFactoryYaml::createAtomsDescription(const YAML::Node & descNode, OptionalDouble atomsRadii) const
{
  std::string sValue;
  unsigned int nAtoms = 1;
  ssc::AtomSpeciesId::Value  specId;

  if(descNode[kw::STRUCTURE__ATOMS__TYPE])
  {
    sValue = descNode[kw::STRUCTURE__ATOMS__TYPE].as< ::std::string>();
    OptionalAtomSpeciesCount atomSpeciesCount = parseAtomTypeString(sValue);
    if(atomSpeciesCount)
    {
      specId = atomSpeciesCount->first;
      nAtoms = atomSpeciesCount->second;
    }
  }

  build_cell::AtomsDescriptionPtr atomsDescription(new build_cell::AtomsDescription(specId, nAtoms));

  // If there is a default atoms radii first set that and then try to see if there
  // is a specific one for this atom
  if(atomsRadii)
    atomsDescription->setRadius(*atomsRadii);

  if(descNode[kw::STRUCTURE__ATOMS__RADIUS])
  {
    atomsDescription->setRadius(descNode[kw::STRUCTURE__ATOMS__RADIUS].as<double>());
  }


  return atomsDescription;
}



::sstbx::build_cell::AtomConstraintDescription *
SsLibFactoryYaml::createAtomConstraintDescription(
  const YAML::Node & descNode) const
{
  // namespace aliases
  namespace ssbc  = ::sstbx::build_cell;
  namespace ssc   = ::sstbx::common;

  ::std::auto_ptr<ssbc::AtomConstraintDescription> constraint(NULL);

  if(descNode[kw::TYPE])
  {
    ::std::string sValue;
    double dValue;
    const ::std::string type = descNode[kw::TYPE].as< ::std::string>();

    //if(type == kw::STR_DESC__CONSTRAINTS__TYPE___MINSEP)
    //{
    //  if(descNode[kw::VALUE])
    //  {
    //    dValue = descNode[kw::VALUE].as<double>();
    //    constraint.reset(new ssbc::Minsep(dValue));
    //  }
    //}
    //else
    //{
      throw FactoryError() << ErrorType(UNRECOGNISED_KEYWORD);
    //}
  
  }
  else
  {
    throw FactoryError() << ErrorType(REQUIRED_KEYWORD_MISSING) << Keyword(kw::TYPE);
  }

  return constraint.release();
}

ssbc::StructureConstraintDescription *
SsLibFactoryYaml::createStructureConstraintDescription(const YAML::Node & descNode) const
{
  // TODO: No structure constraint
  return NULL;
}

SsLibFactoryYaml::OptionalAtomSpeciesCount
SsLibFactoryYaml::parseAtomTypeString(const ::std::string & atomSpecString) const
{
  OptionalAtomSpeciesCount atomSpeciesCount;

  const Tok tok(atomSpecString, tokSep);

  Tok::const_iterator it = tok.begin();

  AtomSpeciesCount type;
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
      type.first = common::AtomSpeciesDatabase::inst().getIdFromSymbol(*it);
      if(type.first != common::AtomSpeciesId::DUMMY)
        successful = true;
    }
  }

  if(successful)
    atomSpeciesCount.reset(type);

  return atomSpeciesCount;
}

}
}


#endif /* SSLIB_USE_YAML */
