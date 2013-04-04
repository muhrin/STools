/*
 * FactoryFwd.h
 *
 *
 *  Created on: May 29, 2012
 *      Author: Martin Uhrin
 */

#ifndef FACTORY_FWD_H
#define FACTORY_FWD_H

// INCLUDES /////////////////////////////////////////////
#include <string>
#include <vector>

#include <boost/optional.hpp>
#include <boost/variant.hpp>

#include <armadillo>

#include "common/AtomSpeciesId.h"
#include "utility/HeterogeneousMap.h"

// DEFINES //////////////////////////////////////////////

namespace sstbx {
namespace factory {

///////////////////////////////////////////////////////////
// TYPEDEFS
///////////////////////////////////////////////////////////
typedef ::std::vector< ::std::string> AtomsCompactInfo;
typedef ::boost::variant<AtomsCompactInfo, utility::HeterogeneousMap> AtomsDataEntry;
typedef ::std::vector<AtomsDataEntry> AtomsDataEntryList;
typedef ::std::string AtomSpeciesIdType;

///////////////////////////////////////////////////////////
// CLASSES
///////////////////////////////////////////////////////////
struct AtomSpeciesCount
{
  AtomSpeciesCount(): count(1) {}
  //common::AtomSpeciesId::Value species;
  AtomSpeciesIdType species;
  unsigned int count;
};

struct MinMax
{
  ::boost::optional<double> min;
  ::boost::optional<double> max;
};

//typedef VectorAsString<common::AtomSpeciesId::Value> AtomSpeciesIdVector;
typedef ::std::vector<AtomSpeciesIdType> AtomSpeciesIdVector;

}
}

#endif /* FACTORY_FWD_H */

