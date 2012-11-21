/*
 * LoadSeedStructures.h
 *
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef LOAD_SEED_STRUCTURES_H
#define LOAD_SEED_STRUCTURES_H

// INCLUDES /////////////////////////////////////////////
#include "StructurePipe.h"

#include <string>

#include <boost/filesystem.hpp>
#include <boost/ptr_container/ptr_list.hpp>

// From SSTbx
#include <SSLib.h>
#include <common/Types.h>
#include <io/ResReaderWriter.h>

// From PipelineLib
#include <pipelib/AbstractSimpleStartBlock.h>

// FORWARD DECLARATIONS ////////////////////////////////////
namespace sstbx {
namespace common {
class AtomSpeciesDatabase;
class Structure;
}
}

namespace spipe {
namespace blocks {

class LoadSeedStructures : public pipelib::AbstractSimpleStartBlock<StructureDataTyp, SharedDataTyp>
{
public:

  /**
  /*  seedStructures - Can be:
  /*  - path to single structure
  /*  - path to folder containing structures
  /*  - wildcard path to structures or folders containig structures
  /*  - a list containing a mixture of any of the above
  /*
  /**/
  LoadSeedStructures(
    const sstbx::common::AtomSpeciesDatabase & atomSpeciesDb,
    const ::std::string & seedStructures);

  // From StartBlock ///
	virtual void start();
  // End from StartBlock

private:

  typedef ::sstbx::common::StructurePtr StructurePtr;
  typedef ::boost::ptr_list<sstbx::common::Structure> StructuresList;

  enum EntryType { UNKNOWN, FILE_PATH, FOLDER_PATH, WILDCARD_PATH };

  int processEntry(const ::std::string & entry);
  int processWildcardEntry(const ::std::string & entry);
  int processFilePath(const boost::filesystem::path & entryPath);
  int processFolderPath(const boost::filesystem::path & entryPath);
  EntryType entryType(const ::std::string & entry) const;

  sstbx::io::ResReaderWriter  myReader;
  StructuresList              myStructures;
  const ::sstbx::common::AtomSpeciesDatabase & mySpeciesDb;
};

}}

#endif /* LOAD_SEED_STRUCTURES_H */
