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

#include <map>

#include <boost/ptr_container/ptr_vector.hpp>

#include <armadillo>

#include <yaml-cpp/yaml.h>

// From SSTbx
#include "SSLibTypes.h"
#include "build_cell/IStructureGenerator.h"
#include "build_cell/StructureDescription.h"
#include "common/SsLibYamlKeywords.h"
#include "io/IStructureWriter.h"
#include "potential/IGeomOptimiser.h"
#include "potential/IPotential.h"
#include "potential/SimplePairPotential.h"
#include "potential/TpsdGeomOptimiser.h"
#include "utility/IStructureComparator.h"
#include "utility/IStructureSet.h"


// Local includes
#include "common/YamlCommon.h"

// FORWARD DECLARATIONS ////////////////////////////////////

namespace sstbx {
namespace build_cell {
class AtomsDescription;
}
}

namespace spipe {
namespace common {

class SsLibFactoryYaml
{
public:

  typedef ::sstbx::UniquePtr< ::sstbx::build_cell::StructureDescription>::Type StructureDescriptionPtr;

  enum ErrorCode
  {
    UNKNOWN,
    BAD_TAG,
    UNRECOGNISED_KEYWORD,
    REQUIRED_KEYWORD_MISSING,
    MALFORMED_VALUE
  };

  SsLibFactoryYaml();

  //::sstbx::build_cell::RandomCellDescription *            createCellDescription(const YAML::Node & desc);
  StructureDescriptionPtr                                 createStructureDescription(const YAML::Node & desc);
  ::sstbx::build_cell::IStructureGenerator *       createCrystalStructureGenerator(const YAML::Node & desc);
  ::sstbx::potential::IPotential *                        createPotential(const YAML::Node & desc);
  ::sstbx::potential::IGeomOptimiser *                    createGeometryOptimiser(const YAML::Node & desc);
  ::sstbx::utility::IStructureComparator *                createStructureComparator(const YAML::Node & node);
  ::sstbx::utility::IStructureSet *                       createStructureSet(const YAML::Node & desc);
  ::sstbx::io::IStructureWriter *                         createStructureWriter(const YAML::Node & node);


private:

  ::sstbx::build_cell::AtomsDescription *                 createAtomsDescription(const YAML::Node & desc) const;
  ::sstbx::build_cell::AtomConstraintDescription *        createAtomConstraintDescription(const YAML::Node & descNode) const;
  ::sstbx::build_cell::StructureConstraintDescription *   createStructureConstraintDescription(const YAML::Node & descNode) const;

  void checkKeyword(const ::spipe::common::sslib_yaml_keywords::KwTyp & kw, const YAML::Node & node) const;


  /**
  /* Containers to hold the pipelines and blocks created by the factory.
  /* The factory retains ownership and will cleanly release these under the
  /* RAII idiom when the factory is destroyed.
  /**/
  //::boost::ptr_vector< ::sstbx::build_cell::RandomCellDescription>             myCellDescriptions;
  ::boost::ptr_vector< ::sstbx::build_cell::IStructureGenerator>        myCrystalStructureGenerators;
  ::boost::ptr_vector< ::sstbx::io::IStructureWriter>                          myStructureWriters;
  ::boost::ptr_vector< ::sstbx::potential::IGeomOptimiser>                     myOptimisers;
  ::boost::ptr_vector< ::sstbx::potential::IPotential>                         myPotentials;
  ::boost::ptr_vector< ::sstbx::utility::IStructureComparator >                myStructureComparators;
  ::boost::ptr_vector< ::sstbx::utility::IStructureSet>                        myStructureSets;
  ::boost::ptr_vector< ::sstbx::build_cell::StructureDescription>              myStructureDescriptions;


};

}
}



#endif /* SS_LIB_FACTORY_YAML_H */
