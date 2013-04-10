/*
 * DataGatherer.h
 *
 *
 *  Created on: Nov 19, 2012
 *      Author: Martin Uhrin
 */

#ifndef DATA_GATHERER_H
#define DATA_GATHERER_H

// INCLUDES /////////////////////////////////////////////
#include <string>

#include <boost/optional.hpp>

// FORWARD DECLARES ////////////////////////////////
namespace sstbx {
namespace common {
class Structure;
}
}

namespace stools {
namespace sinfo {

class DataGatherer
{
public:

  DataGatherer();

  void gather(const ::sstbx::common::Structure & structure);

  ::boost::optional<double> getLowestEnergy() const;
  ::boost::optional<double> getLowestEnergyPerAtom() const;

  ::boost::optional<double> getLowestEnthalpy() const;
  ::boost::optional<double> getLowestEnthalpyPerAtom() const;

private:
  double myLowestEnergy;
  double myLowestEnergyPerAtom;
  double myLowestEnthalpy;
  double myLowestEnthalpyPerAtom;
};

}
}


#endif /* DATA_GATHERER_H */
