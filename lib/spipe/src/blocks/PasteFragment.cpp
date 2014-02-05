/*
 * PasteFragment.cpp
 *
 *  Created on: Feb 5, 2014
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "blocks/PasteFragment.h"

#include <boost/foreach.hpp>
#include <boost/range/iterator_range.hpp>

#include <spl/math/Matrix.h>

// NAMESPACES ////////////////////////////////

namespace spipe {
namespace blocks {

PasteFragment::PasteFragment(const spl::io::ResourceLocator & loc) :
    Block("Paste fragment"), myFragmentLoc(loc), myTransform(arma::eye(4, 4))
{
}

PasteFragment::PasteFragment(const spl::io::ResourceLocator & loc,
    const arma::mat44 & transform) :
    Block("Paste fragment"), myFragmentLoc(loc), myTransform(transform)
{
}

PasteFragment::PasteFragment(const spl::common::Structure & fragment) :
    Block("Paste fragment"), myTransform(arma::eye(4, 4)), myFragment(fragment)
{
}

PasteFragment::PasteFragment(const spl::common::Structure & fragment,
    const arma::mat44 & transform) :
    Block("Paste fragment"), myTransform(transform), myFragment(fragment)
{
}

void
PasteFragment::pipelineInitialising()
{
  if(!myFragmentLoc.empty())
  {
    spl::UniquePtr< spl::common::Structure>::Type structure =
        getEngine()->globalData().getStructureIo().readStructure(myFragmentLoc);
    if(structure.get())
      myFragment = *structure;
  }
}

void
PasteFragment::in(StructureDataType * const data)
{
  if(data->getStructure() && myFragment.getNumAtoms() != 0)
  {
    spl::common::Structure & structure = *data->getStructure();
    BOOST_FOREACH(const spl::common::Atom & atom,
        boost::make_iterator_range(myFragment.atomsBegin(), myFragment.atomsEnd()))
    {
      structure.newAtom(atom).setPosition(
          spl::math::transformCopy(atom.getPosition(), myTransform));
    }
  }
  out(data);
}

}
}

