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
#include <spl/io/ResourceLocator.h>

#include <boost/filesystem.hpp>

#include "utility/InfoToken.h"

// FORWARD DECLARES ////////////////////////////////
namespace spl {
namespace common {
class Structure;
}
namespace io {
class ResourceLocator;
}
}

namespace stools {
namespace utility {

class EnergyToken : public TypedToken< double>
{
public:
  EnergyToken(const ::std::string & name, const ::std::string & symbol,
      const ::std::string & defaultFormatString = "", const bool usePerAtom =
          false);
  EnergyToken(const ::std::string & name, const ::std::string & symbol,
      const double relativeEnergy, const ::std::string & defaultFormatString =
          "", const bool usePerAtom = false);
  void
  setRelativeEnergy(const double relativeEnergy);

protected:
  typedef TypedToken< double>::StructureValue StructureValue;

  virtual StructureValue
  doGetValue(const ::spl::common::Structure & structure) const;

private:
  double myRelativeTo; // The energy that all energies will be relative to
  const bool myUsePerAtom; // Use the energy/atom when comparing
};

class FormulaToken : public TypedToken< ::std::string>
{
public:
  FormulaToken(const ::std::string & name, const ::std::string & symbol,
      const ::std::string & defaultFormatString = "", const bool reduced = false);
protected:
  virtual ::boost::optional< ::std::string>
  doGetValue(const ::spl::common::Structure & structure) const;
private:
  const bool myReduced;
};

namespace functions {
::boost::optional< ::std::string>
getName(const ::spl::common::Structure & structure);
::boost::optional< double>
getVolume(const ::spl::common::Structure & structure);
::boost::optional< double>
getVolumePerAtom(const ::spl::common::Structure & structure);
::boost::optional< double>
getEnergyPerAtom(const ::spl::common::Structure & structure);
::boost::optional< unsigned int>
getNumAtoms(const ::spl::common::Structure & structure);
::boost::optional< ::std::string>
getSpaceGroupSymbol(const ::spl::common::Structure & structure);
::boost::optional< unsigned int>
getSpaceGroupNumber(const ::spl::common::Structure & structure);
::boost::optional< ::spl::io::ResourceLocator>
getRelativeLoadPath(const ::spl::common::Structure & structure);

}

}
}

#endif /* CUSTOM_TOKENS_H */
