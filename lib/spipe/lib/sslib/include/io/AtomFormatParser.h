/*
 * AtomFormatParser.h
 *
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef ATOM_FORMAT_PARSER_H
#define ATOM_FORMAT_PARSER_H

// INCLUDES /////////////////////////////////////////////

#include <string>
#include <vector>

#include <boost/optional.hpp>
#include <boost/variant.hpp>

#include "factory/FactoryFwd.h"
#include "utility/HeterogeneousMap.h"

namespace sstbx {

// FORWARD DECLARATIONS ////////////////////////////////////

namespace io {

class AtomFormatParser
{
public:
  typedef ::std::vector< ::std::string> FormatDescription;

  AtomFormatParser();
  explicit AtomFormatParser(const FormatDescription & formatDescription);

  AtomFormatParser & operator =(const AtomFormatParser & toCopy);
  
  bool setFormat(const FormatDescription & format);

  template <typename T>
  void addEntry(const ::std::string & name, utility::Key<T> & key);

  template <typename T>
  bool setDefault(utility::Key<T> & key, const T & value);

  template <typename T>
  ::boost::optional<T> getValue(const utility::Key<T> & key, const factory::AtomsDataEntry & atomData) const;
  
private:
  typedef ::std::vector<const utility::KeyId *> FormatOrder;
  typedef ::std::map< ::std::string, const utility::KeyId *> FormatEntries;

  void init();

  utility::HeterogeneousMap myDefaults;
  FormatOrder myFormatOrder;
  FormatEntries myFormatEntries;
};

}
}

#include "io/detail/AtomFormatParser.h"

#endif /* ATOM_FORMAT_PARSER_H */
