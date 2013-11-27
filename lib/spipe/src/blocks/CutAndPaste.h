/*
 * CutANdPaste.h
 *
 *
 *  Created on: Nov 24, 2011
 *      Author: Martin Uhrin
 */

#ifndef CUT_AND_PASTE_H
#define CUT_AND_PASTE_H

// INCLUDES /////////////////////////////////////////////
#include "StructurePipe.h"

#include <map>

#include <boost/noncopyable.hpp>

#include <spl/factory/GenShapeFactory.h>

#include "SpTypes.h"

// FORWARD DECLARATIONS ////////////////////////////////////

namespace spipe {
namespace blocks {

class CutAndPaste : public PipeBlock, ::boost::noncopyable
{
  typedef ::spl::factory::GenShapeFactory::GenShapePtr ShapePtr;
public:
  struct Settings
  {
    Settings():
      paste(false)
    {
    }
    bool paste;
  };

  CutAndPaste(ShapePtr shape, Settings & settings);

  // From Block /////////////////
  virtual void
  in(common::StructureData * const data);
  // End from Block /////////////

private:
  ShapePtr myShape;
  const Settings & mySettings;
};

}
}

#endif /* CUT_AND_PASTE_H */