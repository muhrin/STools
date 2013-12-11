/*
 * CutAndPaste.cpp
 *
 *  Created on: Nov 24, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "blocks/CutAndPaste.h"

#include <boost/foreach.hpp>

#include <spl/common/Atom.h>

#include "common/Assert.h"

// NAMESPACES ////////////////////////////////

namespace spipe {
namespace blocks {

namespace splbc = ::spl::build_cell;
namespace splc = ::spl::common;

CutAndPaste::CutAndPaste(ShapePtr shape, Settings & settings):
    Block("Cut and paste"),
    myShape(shape),
    mySettings(settings)
{
  SPIPE_ASSERT(myShape.get());
}

void
CutAndPaste::in(common::StructureData * const data)
{
  if(!data->getStructure())
  {
    out(data);
    return;
  }

  splc::Structure * const structure = data->getStructure();

  ::std::vector<splc::Atom> cutAtoms;
  for(splc::Structure::AtomIterator it = structure->atomsBegin();
      it != structure->atomsEnd(); /* increment in body */)
  {
    if(myShape->isInShape(it->getPosition()))
    {
      cutAtoms.push_back(*it);
      it = structure->eraseAtom(it);
    }
    else
      ++it;
  }

  if(mySettings.paste)
  {
    const size_t numUntouched = structure->getNumAtoms();
    BOOST_FOREACH(splc::Atom & atom, cutAtoms)
    {
      atom.setPosition(myShape->randomPoint());
      structure->newAtom(atom);
    }

    if(mySettings.separate)
    {
      splbc::SeparationData sepData(*structure, getEngine()->globalData().getSpeciesDatabase());
      if(mySettings.fixUntouched)
      {
        for(size_t i = 0; i < numUntouched; ++i)
          sepData.fixedPoints.insert(i);
      }
      mySeparator.separatePoints(&sepData);
      structure->setAtomPositions(sepData.points);
    }
  }

  out(data);
}

}
}
