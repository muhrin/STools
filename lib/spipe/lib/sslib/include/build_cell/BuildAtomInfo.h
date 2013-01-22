/*
 * BuildAtomInfo.h
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef BUILD_ATOM_INFO_H
#define BUILD_ATOM_INFO_H

// INCLUDES ////////////

#include <boost/shared_ptr.hpp>


// DEFINITION ///////////////////////

namespace sstbx {
// FORWARD DECLARATIONS ///////
namespace common {
class Atom;
class AtomGroup;
}

namespace build_cell {

class BuildAtomInfo
{
public:
  typedef ::boost::shared_ptr<common::AtomGroup> AtomGroupPtr;

  BuildAtomInfo(common::Atom & atom, AtomGroupPtr group = AtomGroupPtr());

  common::Atom & getAtom();
  const common::Atom & getAtom() const;

  common::AtomGroup * getGroup();
  const common::AtomGroup * getGroup() const;

  void setFixed(const bool fixed);
  bool isFixed() const;

private:

  common::Atom & myAtom;
  AtomGroupPtr myGroup; 
  bool myFixed;

};

}
}


#endif /* BUILD_ATOM_INFO_H */
