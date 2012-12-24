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

#include <pipelib/pipelib.h>

#include "PipeLibTypes.h"

// FORWARD DECLARATIONS ////////////////////////////////////


namespace sstbx {
namespace io {
struct AdditionalData;
class StructureReadWriteManager;
}
}


namespace spipe {
namespace blocks {


class WriteStructure : public pipelib::PipeBlock< ::spipe::StructureDataTyp, ::spipe::SharedDataTyp, ::spipe::SharedDataTyp>
{
public:
	WriteStructure(const ::sstbx::io::StructureReadWriteManager & writerManager);

  virtual void in(spipe::StructureDataTyp & data);

private:
	const ::sstbx::io::StructureReadWriteManager & myWriterManager;
};

}
}


#endif /* WRITE_STRUCTURE_H */
