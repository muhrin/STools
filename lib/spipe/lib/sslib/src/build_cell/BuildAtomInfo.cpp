/*
 * BuildAtomInfo.cpp
 *
 *  Created on: Nov 10, 2011
 *      Author: Martin Uhrin
 */

// INCLUEDES /////////////
#include "build_cell/BuildAtomInfo.h"

#include "common/Atom.h"

namespace sstbx {
namespace build_cell {

BuildAtomInfo::BuildAtomInfo(common::Atom & atom, AtomGroupPtr group):
myAtom(atom),
myGroup(group),
myFixed(false)
{}

common::Atom & BuildAtomInfo::getAtom()
{
  return myAtom;
}

const common::Atom & BuildAtomInfo::getAtom() const
{
  return myAtom;
}

common::AtomGroup * BuildAtomInfo::getGroup()
{
  return myGroup.get();
}

const common::AtomGroup * BuildAtomInfo::getGroup() const
{
  return myGroup.get();
}

void BuildAtomInfo::setFixed(const bool fixed)
{
  myFixed = fixed;
}

bool BuildAtomInfo::isFixed() const
{
  return myFixed;
}

}
}
