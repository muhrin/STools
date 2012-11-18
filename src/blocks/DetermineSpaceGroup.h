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

#include <pipelib/AbstractSimpleBlock.h>

// FORWARD DECLARATIONS ////////////////////////////////////


namespace spipe
{
namespace blocks
{

  class DetermineSpaceGroup : public pipelib::AbstractSimpleBlock<StructureDataTyp, SharedDataTyp>
{
public:

  DetermineSpaceGroup();

  // From PipeBlock //////////////////////
  virtual void in(StructureDataTyp & data);
  // End from PipeBlock /////////////////

private:

};

}}

#endif /* DETERMINE_SPACE_GROUP_H */
