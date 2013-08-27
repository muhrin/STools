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

#include <spl/io/ResourceLocator.h>

#include "SpTypes.h"

// FORWARD DECLARATIONS ////////////////////////////////////


namespace spl {
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
  static const bool WRITE_MULTI_DEFAULT;

	WriteStructure();

  bool getWriteMulti() const;
  void setWriteMulti(const bool writeMulti);

  const ::std::string & getFileType() const;
  void setFileType(const ::std::string & extension);

  // From PipeBlock ////
  virtual void pipelineStarting();
  virtual void in(StructureDataType & data);
  // End from PipeBlock ////

private:

  struct State
  {
    enum Value { DISABLED, USE_CUSTOM_WRITER, USE_DEFAULT_WRITER };
  };

  ::spl::io::ResourceLocator generateLocator(
    ::spl::common::Structure & structure,
    const ::spl::io::IStructureWriter & writer) const;
  bool useMultiStructure(const ::spl::io::IStructureWriter & writer) const;

  State::Value myState;
  bool myWriteMultiStructure;
  ::std::string myFileType;
};

}
}

#endif /* WRITE_STRUCTURE_H */
