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
#include <boost/shared_ptr.hpp>

// DEFINES ////////////////////////////////////////////////

namespace sstbx
{
namespace common
{

// FORWARD DECLARATIONS ////////////////////////////////////
class AbstractFmidCell;
class Atom;
class Structure;

typedef ::boost::shared_ptr<Atom> AtomPtr;
typedef ::boost::shared_ptr<const Atom> ConstAtomPtr;

typedef ::boost::shared_ptr<Structure> StructurePtr;
typedef ::boost::shared_ptr<const Structure> ConstStructurePtr;

typedef ::boost::shared_ptr<AbstractFmidCell> UnitCellPtr;
typedef ::boost::shared_ptr<const AbstractFmidCell> ConstUnitCellPtr;

}
}

#endif /* TYPES_H */
