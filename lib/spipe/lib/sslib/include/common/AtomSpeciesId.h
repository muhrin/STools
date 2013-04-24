/*
 * AtomSpeciesId.h
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef ATOM_SPECIES_ID_H
#define ATOM_SPECIES_ID_H

// INCLUDES ////////////
#include "utility/Enum.h"

// FORWARD DECLARATIONS ///////

namespace sstbx {
namespace common {

struct AtomSpeciesId : public ::sstbx::utility::Enum<AtomSpeciesId>
{
public:

  typedef ::sstbx::utility::Enum<AtomSpeciesId> Base;

  // Adding this for compatibility with previous use of this type
  typedef AtomSpeciesId Value;

  static const Enumerator DUMMY;
  static const Enumerator H;
  static const Enumerator HE;
  static const Enumerator LI;
  static const Enumerator BE;
  static const Enumerator B;
  static const Enumerator C;
  static const Enumerator N;
  static const Enumerator O;
  static const Enumerator F;
  static const Enumerator NE;
  static const Enumerator NA;
  static const Enumerator MG;
  static const Enumerator AL;
  static const Enumerator SI;
  static const Enumerator P;
  static const Enumerator CL;
  static const Enumerator FE;
  static const Enumerator NI;
  static const Enumerator AS;
  static const Enumerator CUSTOM_1;
  static const Enumerator CUSTOM_2;
  static const Enumerator CUSTOM_3;
  static const Enumerator CUSTOM_4;
  static const Enumerator CUSTOM_5;

  //AtomSpeciesId(const ::std::string & symbol);

  //const ::std::string & toString() const;

private:
  //::std::string mySymbol;
};

}
}


#endif /* ATOM_SPECIES_IDS_H */
