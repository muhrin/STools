/*
 * CutAndPaste.h
 *
 *
 *  Created on: Nov 24, 2013
 *      Author: Martin Uhrin
 */

#ifndef CUT_AND_PASTE_H
#define CUT_AND_PASTE_H

// INCLUDES /////////////////////////////////////////////
#include "StructurePipe.h"

#include <map>

#include <boost/noncopyable.hpp>

#include <spl/build_cell/PointSeparator.h>
#include <spl/common/AtomSpeciesDatabase.h>
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
      paste(false), separate(true), fixUntouched(true)
    {
    }
    bool paste;
    bool separate;
    bool fixUntouched;
  };

  CutAndPaste(ShapePtr shape, Settings & settings);

  void
  setPairDistances(const spl::factory::PairDistances & distances);

  // From Block /////////////////
  virtual void
  pipelineInitialised();
  virtual void
  in(common::StructureData * const data);
  // End from Block /////////////

private:
  ShapePtr myShape;
  const Settings mySettings;
  const ::spl::build_cell::PointSeparator mySeparator;

  // TODO: Temp, probably remove these later
  spl::factory::PairDistances myPairDistances;
  spl::common::AtomSpeciesDatabase mySpeciesDb;
};

}
}

#endif /* CUT_AND_PASTE_H */
