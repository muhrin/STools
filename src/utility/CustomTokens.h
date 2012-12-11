/*
 * CustomTokens.h
 *
 *
 *  Created on: Nov 19, 2012
 *      Author: Martin Uhrin
 */

#ifndef CUSTOM_TOKENS_H
#define CUSTOM_TOKENS_H

// INCLUDES /////////////////////////////////////////////
#include "utility/InfoToken.h"

// FORWARD DECLARES ////////////////////////////////
namespace sstbx {
namespace common {
class Structure;
}
}

namespace stools {
namespace utility {

class EnergyToken : public TypedToken<double>
{
public:
  EnergyToken(
    const ::std::string & name,
    const ::std::string & symbol,
    const bool usePerAtom = false);

  EnergyToken(
    const ::std::string & name,
    const ::std::string & symbol,
    const double relativeEnergy,
    const bool usePerAtom = false);

  void setRelativeEnergy(const double relativeEnergy);

protected:
  typedef TypedToken<double>::StructureValue StructureValue;

  virtual StructureValue doGetValue(const ::sstbx::common::Structure & structure) const;

private:

  double myRelativeTo; // The energy that all energies will be relative to
  const bool myUsePerAtom;  // Use the energy/atom when comparing
};


namespace functions {
::boost::optional< ::std::string> getName(const ::sstbx::common::Structure & structure);
::boost::optional<double> getVolume(const ::sstbx::common::Structure & structure);
::boost::optional<double> getEnergyPerAtom(const ::sstbx::common::Structure & structure);
::boost::optional<unsigned int> getNumAtoms(const ::sstbx::common::Structure & structure);
}

}
}


#endif /* CUSTOM_TOKENS_H */
