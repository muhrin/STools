/*
 * WriteStructures.h
 * Write structures out to file.
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef WRITE_STRUCTURES_H
#define WRITE_STRUCTURES_H

// INCLUDES /////////////////////////////////////////////
#include "StructurePipe.h"

#include <string>

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

class WriteStructures : public PipeBlock, ::boost::noncopyable
{
public:
  static const bool WRITE_MULTI_DEFAULT;
  static const ::std::string FORMAT_DEFAULT;

  WriteStructures();

  bool
  getWriteMulti() const;
  void
  setWriteMulti(const bool writeMulti);

  const ::std::string &
  getFileType() const;
  void
  setFileType(const ::std::string & extension);

  // From PipeBlock ////
  virtual void
  pipelineStarting();
  virtual void
  in(StructureDataType * const data);
  // End from PipeBlock ////

private:
  struct State
  {
    enum Value
    {
      DISABLED, USE_CUSTOM_WRITER, USE_DEFAULT_WRITER
    };
  };

  ::spl::io::ResourceLocator
  generateLocator(::spl::common::Structure & structure,
      const ::spl::io::IStructureWriter & writer) const;
  bool
  useMultiStructure(const ::spl::io::IStructureWriter & writer) const;

  State::Value myState;
  bool myWriteMultiStructure;
  const bool myMultiStructureFromPath;
  ::std::string myFileType;
};

}
}

#endif /* WRITE_STRUCTURE_H */
