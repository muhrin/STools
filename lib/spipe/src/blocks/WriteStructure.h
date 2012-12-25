/*
 * WriteStructure.h
 * Write structures out to file.
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef WRITE_STRUCTURE_H
#define WRITE_STRUCTURE_H

// INCLUDES /////////////////////////////////////////////
#include "StructurePipe.h"

#include <boost/noncopyable.hpp>

#include <pipelib/pipelib.h>

#include "SpTypes.h"

// FORWARD DECLARATIONS ////////////////////////////////////


namespace sstbx {
namespace io {
struct AdditionalData;
class StructureReadWriteManager;
}
}


namespace spipe {
namespace blocks {


class WriteStructure : public SpPipeBlock, ::boost::noncopyable
{
public:
	WriteStructure();

  // From PipeBlock ////
  virtual void in(StructureDataType & data);
  // End from PipeBlock ////
};

}
}


#endif /* WRITE_STRUCTURE_H */
