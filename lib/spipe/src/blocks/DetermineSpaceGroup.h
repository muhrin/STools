/*
 * DetermineSpaceGroup.h
 *
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef DETERMINE_SPACE_GROUP_H
#define DETERMINE_SPACE_GROUP_H

// INCLUDES /////////////////////////////////////////////
#include "StructurePipe.h"

#include <boost/noncopyable.hpp>

#include <pipelib/pipelib.h>

#include "PipeLibTypes.h"

// FORWARD DECLARATIONS ////////////////////////////////////


namespace spipe {
namespace blocks {

class DetermineSpaceGroup : public SpPipeBlock, ::boost::noncopyable
{
public:

  DetermineSpaceGroup();

  // From PipeBlock //////////////////////
  virtual void in(StructureDataTyp & data);
  // End from PipeBlock /////////////////
};

}}

#endif /* DETERMINE_SPACE_GROUP_H */
