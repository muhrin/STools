/*
 * SsLibFactoryYaml.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "common/SsLibFactoryYaml.h"

#include <memory>

#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>

// From SSTbx
#include "build_cell/AtomsDescription.h"
#include "build_cell/AtomConstraintDescription.h"
#include "build_cell/ICellGenerator.h"
#include "build_cell/DefaultCrystalGenerator.h"
#include "build_cell/Minsep.h"
#include "build_cell/RandomCellGenerator.h"
#include "build_cell/StructureConstraintDescription.h"
#include "common/AtomSpeciesDatabase.h"
#include "common/AtomSpeciesId.h"
#include "io/ResReaderWriter.h"
#include "utility/SortedDistanceComparator.h"
#include "utility/UniqueStructureSet.h"

// Local includes
#include "common/FactoryError.h"



// NAMESPACES ////////////////////////////////

namespace spipe { namespace common {

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

namespace kw = ::spipe::common::sslib_yaml_keywords;

SsLibFactoryYaml::SsLibFactoryYaml()
{
}

::sstbx::build_cell::RandomCellDescription<double> *
SsLibFactoryYaml::createCellDescription(const YAML::Node & node)
{
  //// Make sure we have a cell description node
  //if(node.Scalar() != kw::CELL_DESC)
  //{
  //  throw FactoryError() << ErrorType(BAD_TAG) << NodeName(node.Scalar()) << Keyword(kw::CELL_DESC);
  //}

  ssbc::RandomCellDescription<double> * const desc = new ssbc::RandomCellDescription<double>();
  myCellDescriptions.push_back(desc);

  double dValue;

  if(node[kw::CELL_DESC__PARAMS])
  {
    const YAML::Node & paramsNode = node[kw::CELL_DESC__PARAMS];

    if(paramsNode.IsSequence() && paramsNode.size() == 6)
    {
      double params[6];
      for(size_t i = 0; i < 6; ++i)
      {
        params[i] = paramsNode[i].as<double>();
      }
      desc->setLatticeParams(params);
    }
    else
    {
      throw FactoryError() << ErrorType(MALFORMED_VALUE);
    }
  }

  if(node[kw::CELL_DESC__VOL])
  {
    dValue = node[kw::CELL_DESC__VOL].as<double>();
    desc->myVolume.reset(dValue);
  }

  if(node[kw::CELL_DESC__MIN_ANGLE])
  {
    dValue = node[kw::CELL_DESC__MIN_ANGLE].as<double>();
    desc->myMinAngle.reset(dValue);
  }

  if(node[kw::CELL_DESC__MAX_ANGLE])
  {
    dValue = node[kw::CELL_DESC__MAX_ANGLE].as<double>();
    desc->myMaxAngle.reset(dValue);
  }

  return desc;
}


ssbc::StructureDescription * SsLibFactoryYaml::createStructureDescription(const YAML::Node & node)
{
  //// Make sure we have a structure description node
  //if(node.Scalar() != kw::STR_DESC)
  //{
  //  throw FactoryError() << ErrorType(BAD_TAG) << NodeName(node.Scalar());
  //}

  ::std::auto_ptr<ssbc::StructureDescription> strDesc(new ssbc::StructureDescription());

  // Atoms //
  if(node[kw::STR_DESC__ATOMS])
  {
    const YAML::Node & atomsNode = node[kw::STR_DESC__ATOMS];

    if(atomsNode.IsSequence())
    {
      BOOST_FOREACH(const YAML::Node & atomNode, atomsNode)
      {
        strDesc->addChild(createAtomsDescription(atomNode));
      }
    }
    else if(atomsNode.IsMap())
    {
      // Maybe it's a single atom
      strDesc->addChild(createAtomsDescription(atomsNode));  
    }
    else
    {
      throw FactoryError() << ErrorType(MALFORMED_VALUE);
    }
  }

  // Constraints //
  if(node[kw::STR_DESC__CONSTRAINTS])
  {
    const YAML::Node & consNode = node[kw::STR_DESC__CONSTRAINTS];

    ssbc::AtomConstraintDescription * atomsConstraint = NULL;
    ssbc::StructureConstraintDescription * structureConstraint = NULL;

    if(consNode.IsSequence())
    {
      BOOST_FOREACH(const YAML::Node & constraint, consNode)
      {

        // Check if it is a structure constraint
        structureConstraint = createStructureConstraintDescription(constraint);
        if(structureConstraint)
        {
          strDesc->addStructureConstraint(structureConstraint);
        }
        else
        {
          // Check if it's an atom constraint
          atomsConstraint = createAtomConstraintDescription(constraint);
          if(atomsConstraint)
          {
            strDesc->addAtomConstraint(atomsConstraint);
          }
        }
        
      }
    }
    else if(consNode.IsMap())
    {
        // Check if it is a structure constraint
        structureConstraint = createStructureConstraintDescription(consNode);
        if(structureConstraint)
        {
          strDesc->addStructureConstraint(structureConstraint);
        }
        else
        {
          // Check if it's an atom constraint
          atomsConstraint = createAtomConstraintDescription(consNode);
          if(atomsConstraint)
          {
            strDesc->addAtomConstraint(atomsConstraint);
          }
        }
    }
    else
    {
      throw FactoryError() << ErrorType(MALFORMED_VALUE);
    }
  }

  ssbc::StructureDescription * const strPtr = strDesc.release();
  myStructureDescriptions.push_back(strPtr);

  // Assign the pointer so the caller gets the object
  return strPtr;
}

::sstbx::build_cell::ICellGenerator<double> *
SsLibFactoryYaml::createCellGenerator(const YAML::Node & node)
{
  //// Make sure we have a cell generator node
  //if(node.Scalar() != kw::CELL_GENERATOR)
  //{
  //  throw FactoryError() << ErrorType(BAD_TAG) << NodeName(node.Scalar());
  //}

  ssbc::ICellGenerator<double> * generator = NULL;

  if(node[kw::TYPE])
  {
    const ::std::string generatorType = node[kw::TYPE].as< ::std::string>();

    if(generatorType == kw::CELL_GENERATOR__TYPE___DEFAULT)
    {
      generator = new ssbc::RandomCellGenerator<double>();
      myCellGenerators.push_back(generator);
    }
  }
  else
  {
    throw FactoryError() << ErrorType(REQUIRED_KEYWORD_MISSING) << Keyword(kw::TYPE);
  }

  return generator;
}

ssbc::ICrystalStructureGenerator *
SsLibFactoryYaml::createCrystalStructureGenerator(const YAML::Node & node)
{
  //// Make sure we have a structure generator node
  //if(node.Scalar() != kw::STR_GENERATOR)
  //{
  //  throw FactoryError() << ErrorType(BAD_TAG) << NodeName(node.Scalar());
  //}

  // First try to create the unit cell generator

  ssbc::ICellGenerator<double> * cellGenerator = NULL;

  if(node[kw::CELL_GENERATOR])
  {
    cellGenerator = createCellGenerator(node[kw::CELL_GENERATOR]);
  }
  else
  {
    throw FactoryError() << ErrorType(REQUIRED_KEYWORD_MISSING) << Keyword(kw::CELL_GENERATOR);
  }

  ssbc::ICrystalStructureGenerator * generator = NULL;

  if(node[kw::TYPE])
  {
    const ::std::string generatorType = node[kw::TYPE].as< ::std::string>();

    if(generatorType == kw::STR_GENERATOR__TYPE___DEFAULT)
    {
      generator = new ssbc::DefaultCrystalGenerator(*cellGenerator);
      myCrystalStructureGenerators.push_back(generator);
    }
  }
  else
  {
    throw FactoryError() << ErrorType(REQUIRED_KEYWORD_MISSING) << Keyword(kw::TYPE);
  }


  return generator;
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
    const ssp::SimplePairPotential<double>::CombiningRule comb =
      node[kw::POTENTIAL__PAIR_POT__COMB].as<ssp::SimplePairPotential<double>::CombiningRule>();

    const size_t numSpecies = epsilon.mat.n_rows;

    if((numSpecies != sigma.mat.n_rows) || (numSpecies != beta.mat.n_rows))
    {
      throw FactoryError() << ErrorType(MALFORMED_VALUE);
    }

    pot = new ssp::SimplePairPotential<double>(
      numSpecies,
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
      opt = new ssp::TpsdGeomOptimiser<double>(*pp);
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
  const ::spipe::common::sslib_yaml_keywords::KwTyp & kw,
  const YAML::Node & node) const
{
  if(!node[kw])
    throw FactoryError() << ErrorType(REQUIRED_KEYWORD_MISSING);
}

ssbc::AtomsDescription *
SsLibFactoryYaml::createAtomsDescription(const YAML::Node & descNode) const
{
  // namespace aliases
  namespace ssbc  = ::sstbx::build_cell;
  namespace ssc   = ::sstbx::common;

  namespace kw = ::spipe::common::sslib_yaml_keywords;

  std::string sValue;
  int nAtoms = 0;
  ssc::AtomSpeciesId::Value  specId;

  if(descNode[kw::STR_DESC__ATOMS__N])
  {
    nAtoms = descNode[kw::STR_DESC__ATOMS__N].as<int>();
  }
  else
  {
    throw FactoryError() << ErrorType(REQUIRED_KEYWORD_MISSING) << Keyword(kw::STR_DESC__ATOMS__N);
  }

  if(descNode[kw::STR_DESC__ATOMS__SPEC])
  {
    sValue = descNode[kw::STR_DESC__ATOMS__SPEC].as<std::string>();

    specId = ssc::AtomSpeciesDatabase::inst().getIdFromSymbol(sValue);
    if(specId == ssc::AtomSpeciesId::DUMMY)
    {
      throw FactoryError() << ErrorType(MALFORMED_VALUE);
    }
  }
  else
  {
    throw FactoryError() << ErrorType(REQUIRED_KEYWORD_MISSING) << Keyword(kw::STR_DESC__ATOMS__SPEC);
  }


  return new ssbc::AtomsDescription(specId, nAtoms);
}



::sstbx::build_cell::AtomConstraintDescription *
SsLibFactoryYaml::createAtomConstraintDescription(
  const YAML::Node & descNode) const
{
  // namespace aliases
  namespace ssbc  = ::sstbx::build_cell;
  namespace ssc   = ::sstbx::common;

  namespace kw = ::spipe::common::sslib_yaml_keywords;

  ::std::auto_ptr<ssbc::AtomConstraintDescription> constraint(NULL);

  if(descNode[kw::TYPE])
  {
    ::std::string sValue;
    double dValue;
    const ::std::string type = descNode[kw::TYPE].as< ::std::string>();

    if(type == kw::STR_DESC__CONSTRAINTS__TYPE___MINSEP)
    {
      if(descNode[kw::VALUE])
      {
        dValue = descNode[kw::VALUE].as<double>();
        constraint.reset(new ssbc::Minsep(dValue));
      }
    }
    else
    {
      throw FactoryError() << ErrorType(UNRECOGNISED_KEYWORD);
    }
  
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



}
}
