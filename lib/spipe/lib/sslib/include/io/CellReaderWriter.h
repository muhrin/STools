/*
 * CellReaderWriter.h
 *
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef CELL_READER_WRITER_H
#define CELL_READER_WRITER_H

// INCLUDES /////////////////////////////////////////////
#include "io/IStructureReader.h"
#include "io/IStructureWriter.h"

#include <ostream>
#include <vector>


namespace sstbx {
// FORWARD DECLARATIONS ////////////////////////////////////
namespace common {
class AtomSpeciesDatabase;
class Structure;
class UnitCell;
}

namespace io {

class CellReaderWriter :  public virtual IStructureWriter/*,  public virtual IStructureReader*/
{
public:

  static const unsigned int DIGITS_AFTER_DECIMAL;

	virtual ::std::vector<std::string> getSupportedFileExtensions() const;

  // From IStructureReader //
  virtual ::sstbx::common::types::StructurePtr readStructure(
    const ResourceLocator & resourceLocator,
		const ::sstbx::common::AtomSpeciesDatabase & speciesDb) const;

  virtual size_t readStructures(
    StructuresContainer & outStructures,
		const ResourceLocator & resourceLocator,
		const common::AtomSpeciesDatabase & speciesDb) const;
  // End from IStructureReader //

  // from IStructureWriter //
	virtual void writeStructure(
		::sstbx::common::Structure & str,
		const ResourceLocator & locator,
		const common::AtomSpeciesDatabase & speciesDb) const;
  // End from IStructureWriter

  // Single structure per file
  virtual bool multiStructureSupport() const { return false; }

  virtual void writeStructure(
    ::std::ostream & os,
    common::Structure & structure,
    const common::AtomSpeciesDatabase & speciesDb
  ) const;

private:
  void writeLatticeBlock(::std::ostream & os, const common::UnitCell & unitCell) const;
  void writePositionsBlock(
    ::std::ostream & os,
    const common::Structure & structure,
    const common::UnitCell & unitCell,
    const common::AtomSpeciesDatabase & speciesDb
  ) const;
};

}
}

#endif /* CELL_READER_WRITER_H */
