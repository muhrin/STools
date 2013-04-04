/*
 * IFragmentGenerator.h
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef I_FRAGMENT_GENERATOR_H
#define I_FRAGMENT_GENERATOR_H

// INCLUDES ////////////
#include "build_cell/BuildCellFwd.h"
#include "utility/SharedHandle.h"

// DEFINITION ///////////////////////

namespace sstbx {
namespace common {
class AtomSpeciesDatabase;
}
namespace build_cell {
// FORWARD DECLARATIONS ///////
class GenerationOutcome;
class StructureBuild;
class StructureContents;

class IFragmentGenerator
{
protected:
  typedef ptrdiff_t GenerationTicketId;
public:
  typedef utility::SharedHandle<GenerationTicketId, IFragmentGenerator> GenerationTicket;

  virtual ~IFragmentGenerator() {}

  virtual GenerationOutcome generateFragment(
    StructureBuild & build,
    const GenerationTicket ticket,
    const common::AtomSpeciesDatabase & speciesDb
  ) const = 0;

  virtual GenerationTicket getTicket() const = 0;
  virtual StructureContents getGenerationContents(
    const GenerationTicket ticket,
    const common::AtomSpeciesDatabase & speciesDb
  ) const = 0;

  virtual void handleReleased(const GenerationTicketId & id) = 0;

  virtual IFragmentGeneratorPtr clone() const = 0;
};

/**
/*  Support for boost ptr_container copy construction.
/**/
inline IFragmentGenerator * new_clone(const IFragmentGenerator & toClone)
{
  return toClone.clone().release();
}

}
}


#endif /* I_FRAGMENT_GENERATOR_H */
