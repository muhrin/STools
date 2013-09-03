/*
 * NiggliReduce.h
 *
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef NIGGLI_REDUCE_H
#define NIGGLI_REDUCE_H

// INCLUDES /////////////////////////////////////////////
#include "StructurePipe.h"

#include <boost/noncopyable.hpp>

#include <pipelib/pipelib.h>

#include "SpTypes.h"

// FORWARD DECLARATIONS ////////////////////////////////////

namespace spipe {
namespace blocks {

class NiggliReduce : public PipeBlock, ::boost::noncopyable
{
public:
  NiggliReduce();

  virtual void
  in(common::StructureData * const data);
};

}
}

#endif /* NIGGLI_REDUCE_H */
