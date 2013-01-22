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

// From SSTbx
#include <SSLib.h>
#include <common/Types.h>
#include <io/ResReaderWriter.h>

#include <pipelib/pipelib.h>

#include "SpTypes.h"

// FORWARD DECLARATIONS ////////////////////////////////////
namespace sstbx {
namespace common {
class AtomSpeciesDatabase;
class Structure;
}
namespace io {
class ResourceLocator;
}
}

namespace spipe {
namespace blocks {

class LoadSeedStructures : public SpStartBlock, ::boost::noncopyable
{
public:

  static const double ATOMIC_VOLUME_MULTIPLIER;

  /**
  /*  seedStructures - Can be:
  /*  - path to single structure
  /*  - path to folder containing structures
  /*  - wildcard path to structures or folders containig structures
  /*  - a list containing a mixture of any of the above
  /*
  /**/
  LoadSeedStructures(
    const ::std::string & seedStructures,
    const bool tryToScaleVolumes = true);

  // From StartBlock ///
  virtual void pipelineInitialising();
	virtual void start();
  // End from StartBlock

private:

  typedef ::sstbx::common::StructurePtr StructurePtr;
  typedef ::sstbx::io::StructuresContainer StructuresContainer;

  enum EntryType { UNKNOWN, FILE_PATH, FOLDER_PATH, WILDCARD_PATH };

  int processEntry(const ::std::string & entry);
  int processWildcardEntry(const ::std::string & entry);
  int processFileOrFolder(const ::sstbx::io::ResourceLocator & loc);
  EntryType entryType(const ::std::string & entry) const;

  double getTotalAtomicVolume(const ::sstbx::common::Structure & structure) const;

  const ::std::string         mySeedStructuresString;
  StructuresContainer         myStructures;
  const bool                  myTryToScaleVolumes;
};

}
}

#endif /* LOAD_SEED_STRUCTURES_H */
