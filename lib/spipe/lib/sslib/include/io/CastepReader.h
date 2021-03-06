/*
 * CastepReader.h
 *
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef CASTEP_READER_H
#define CASTEP_READER_H

// INCLUDES /////////////////////////////////////////////
#include "io/IStructureReader.h"

#include <istream>

#include <boost/regex.hpp>

#include "OptionalTypes.h"

namespace sstbx {
// FORWARD DECLARATIONS ////////////////////////////////////
namespace common {
class AtomSpeciesDatabase;
}

namespace io {

class CastepReader :  public IStructureReader
{
public:
	virtual ::std::vector<std::string> getSupportedFileExtensions() const;

  // From IStructureReader //
  virtual common::types::StructurePtr readStructure(
    const ResourceLocator & resourceLocator,
		const common::AtomSpeciesDatabase & speciesDb
  ) const;
  virtual size_t readStructures(
    StructuresContainer & outStructures,
		const ResourceLocator & locator,
		const common::AtomSpeciesDatabase & speciesDb
  ) const;
  // End from IStructureReader //

  virtual common::types::StructurePtr readStructure(
    ::std::istream & inputStream,
		const common::AtomSpeciesDatabase & speciesDb,
    const ::std::string & id = ""
  ) const;
  virtual size_t readStructures(
    StructuresContainer & outStructures,
		::std::istream & inputStream,
		const common::AtomSpeciesDatabase & speciesDb
  ) const;

  // Multiple structures per file
  virtual bool multiStructureSupport() const { return true; }

private:

  struct AuxInfo
  {
    OptionalDouble pressure;
    OptionalDouble enthalpy;
    OptionalArmaMat33 stressTensor;
  };

  static const ::std::string CELL_TITLE;
  static const ::std::string LATTICE_PARAMS_TITLE;
  static const ::std::string CONTENTS_TITLE;
  static const ::std::string CONTENTS_BOX_BEGIN;

  bool parseCell(common::UnitCell & unitCell, ::std::istream & inputStream) const;
  bool parseContents(
    common::Structure & structure,
    ::std::istream & inputStream,
    const common::AtomSpeciesDatabase & speciesDb
  ) const;
  bool parseAuxInfo(
    AuxInfo & auxInfo,
    ::std::istream & inputStream,
    const ::std::string & line
  ) const;
  void updateStructure(common::Structure & structure, const AuxInfo & auxInfo) const;
  bool parseStressTensorBox(AuxInfo & auxInfo, ::std::istream & inputStream) const;
  bool parseOptimisationTable(AuxInfo & auxInfo, ::std::istream & inputStream) const;
  bool inBox(const ::std::string & line) const;
};

}
}

#endif /* CASTEP_READER_H */
