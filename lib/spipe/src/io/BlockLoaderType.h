/*
 * BlockLoaderType.h
 *
 *
 *  Created on: Jan 13, 2014
 *      Author: Martin Uhrin
 */

#ifndef BLOCK_LOADER_TYPE_H
#define BLOCK_LOADER_TYPE_H

// INCLUDES /////////////////////////////////////////////
#include <schemer/Map.h>

#include "SpTypes.h"

// DEFINES //////////////////////////////////////////////

namespace spipe {
namespace io {

class BlockLoaderType : public schemer::HeteroMap< BlockHandle>
{
public:
  virtual bool
  valueToNode(const BindingType & map, YAML::Node * const node) const
  {
    // There's currently no way to convert a pipe to the node representation
    return false;
  }
  virtual bool
  nodeToValue(const YAML::Node & node, BindingType * const map,
      schemer::ParseLog * const log) const;
};

}
}

#endif /* BLOCK_LOADER_TYPE_H */

