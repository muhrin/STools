/*
 * XyzReaderWriter.h
 *
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef XYZ_READER_WRITER_H
#define XYZ_READER_WRITER_H

// INCLUDES /////////////////////////////////////////////
#include "io/IStructureWriter.h"

#include <vector>

// FORWARD DECLARATIONS ////////////////////////////////////


namespace sstbx {
namespace io {

class XyzReaderWriter :  public virtual IStructureWriter
{
public:

  static const unsigned int DIGITS_AFTER_DECIMAL;

	/**
	/* Write a structure out to disk.
	/* The user can supply their own species database, however it is up to them
	/* to make sure that the implementation is thread safe if necessary.
	/**/
	virtual void writeStructure(
		::sstbx::common::Structure & str,
		const ResourceLocator & locator,
		const common::AtomSpeciesDatabase & speciesDb) const;


	virtual ::std::vector<std::string> getSupportedFileExtensions() const;

  /**
  /* Does this reader support reading multiple structures from a single file.
  /**/
  virtual bool multiStructureSupport() const;
};

}
}

#endif /* XYZ_READER_WRITER_H */
