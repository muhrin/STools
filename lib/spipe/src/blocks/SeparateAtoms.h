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

// FORWARD DECLARATIONS ////////////////////////////////////

namespace spipe {
namespace blocks {

class SeparateAtoms : public PipeBlock, ::boost::noncopyable
{
public:
  SeparateAtoms();

  virtual void
  in(StructureDataType * const data);
};

}
}

#endif /* SEPARATE_ATOMS_H */
