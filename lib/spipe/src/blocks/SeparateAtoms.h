/*
 * SeparateAtoms.h
 *
 *  Created on: Jan 23, 2014
 *      Author: Martin Uhrin
 */

#ifndef SEPARATE_ATOMS_H
#define SEPARATE_ATOMS_H

// INCLUDES /////////////////////////////////////////////
#include "StructurePipe.h"

#include "SpTypes.h"

#include <spl/factory/FactoryFwd.h>
// FORWARD DECLARATIONS ////////////////////////////////////

namespace spipe {
namespace blocks {

class SeparateAtoms : public PipeBlock, ::boost::noncopyable
{
public:
  SeparateAtoms();

  void
  setPairDistances(const spl::factory::PairDistances & distances);

  virtual void
  pipelineInitialised();
  virtual void
  in(StructureDataType * const data);
private:
  spl::factory::PairDistances myPairDistances;
  spl::common::AtomSpeciesDatabase mySpeciesDb;
};

}
}

#endif /* SEPARATE_ATOMS_H */
