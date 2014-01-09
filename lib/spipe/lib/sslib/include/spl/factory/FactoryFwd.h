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
#include <iostream>
#include <string>
#include <vector>

#include <boost/optional.hpp>
#include <boost/variant.hpp>

#include <armadillo>

#include "spl/build_cell/AtomsDescription.h"
#include "spl/common/AtomSpeciesId.h"
#include "spl/utility/HeterogeneousMap.h"

// DEFINES //////////////////////////////////////////////

namespace spl {
namespace factory {

///////////////////////////////////////////////////////////
// TYPEDEFS
///////////////////////////////////////////////////////////
typedef ::std::map< ::std::string, double> PairDistances;

///////////////////////////////////////////////////////////
// CLASSES
///////////////////////////////////////////////////////////
struct AtomSpeciesCount
{
  AtomSpeciesCount() :
      count(1)
  {
  }
  ::std::string species;
  build_cell::AtomsDescription::CountRange count;
};

std::ostream &
operator <<(std::ostream & out, const AtomSpeciesCount & speciesCount)
{
  out << speciesCount.species << " " << speciesCount.count;
  return out;
}

std::istream &
operator >>(std::istream &in, AtomSpeciesCount & speciesCount)
{
  in >> speciesCount.species >> speciesCount.count;
  return in;
}

struct MinMax
{
  ::boost::optional< double> min;
  ::boost::optional< double> max;
};

}
}

#endif /* FACTORY_FWD_H */

