/*
 * PasteFragment.h
 *
 *  Created on: Feb 5, 2014
 *      Author: Martin Uhrin
 */

#ifndef PASTE_FRAGMENT_H
#define PASTE_FRAGMENT_H

// INCLUDES /////////////////////////////////////////////
#include "StructurePipe.h"

#include <boost/noncopyable.hpp>

#include <armadillo>

#include <spl/common/Structure.h>
#include <spl/io/ResourceLocator.h>

#include "SpTypes.h"

// FORWARD DECLARATIONS ////////////////////////////////////

namespace spl {
namespace common {
class Structure;
}
}

namespace spipe {
namespace blocks {

class PasteFragment : public PipeBlock, boost::noncopyable
{
public:
  PasteFragment(const spl::io::ResourceLocator & loc);
  PasteFragment(const spl::io::ResourceLocator & loc,
      const arma::mat44 & transform);
  PasteFragment(const spl::common::Structure & fragment);
  PasteFragment(const spl::common::Structure & fragment,
      const arma::mat44 & transform);

  virtual void
  pipelineInitialising();
  virtual void
  in(StructureDataType * const data);

private:
  spl::io::ResourceLocator myFragmentLoc;
  arma::mat44 myTransform;
  spl::common::Structure myFragment;

};

}
}

#endif /* PASTE_FRAGMENT_H */
