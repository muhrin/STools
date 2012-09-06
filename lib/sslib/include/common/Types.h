/*
 * Types.h
 *
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef TYPES_H
#define TYPES_H

// INCLUDES /////////////////////////////////////////////
#include <boost/optional.hpp>
#include <boost/shared_ptr.hpp>

#include <armadillo>

// DEFINES ////////////////////////////////////////////////

namespace sstbx {
namespace common {

// FORWARD DECLARATIONS ////////////////////////////////////
class Atom;
class Structure;
class UnitCell;

typedef ::boost::shared_ptr<Atom> AtomPtr;
typedef ::boost::shared_ptr<const Atom> ConstAtomPtr;

typedef ::boost::shared_ptr<Structure> StructurePtr;
typedef ::boost::shared_ptr<const Structure> ConstStructurePtr;

typedef ::boost::shared_ptr<UnitCell> UnitCellPtr;
typedef ::boost::shared_ptr<const UnitCell> ConstUnitCellPtr;

typedef ::boost::optional<const ::arma::mat33> OptionalConstMat33;

}
}

#endif /* TYPES_H */
