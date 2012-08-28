/*
 * AtomExtruder.h
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef ATOM_EXTRUDER_H
#define ATOM_EXTRUDER_H

// INCLUDES ////////////
#include "build_cell/StructureDescriptionVisitor.h"

#include <vector>

#include <armadillo>

#include "common/Types.h"

namespace sstbx {

namespace common {
  class AbstractFmidCell;
  class Structure;
}
  
namespace build_cell {

class AtomExtruder
{
public:

  bool extrudeAtoms(common::Structure & structure) const;

private:

  bool extrudeAtoms(
    const common::AbstractFmidCell & cell,
    ::std::vector<common::AtomPtr> & atoms,
    const ::arma::mat & sepSqMtx,
    const size_t maxIterations = 10000) const;

};

}
}

#endif /* ATOM_EXTRUDER_H */
