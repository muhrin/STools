/*
 * BlockLoader.h
 *
 *
 *  Created on: Dec 13, 2013
 *      Author: Martin Uhrin
 */

#ifndef BLOCK_LOADER_H
#define BLOCK_LOADER_H

// INCLUDES /////////////////////////////////////////////
#include "StructurePipe.h"

#include <iostream>
#include <map>
#include <memory>
#include <string>

#include <boost/ptr_container/ptr_map.hpp>

#include <spl/common/AtomSpeciesDatabase.h>

#include "SpTypes.h"
#include "factory/BlockFactory.h"

// FORWARD DECLARATIONS ////////////////////////////////////
namespace spipe {
namespace build {

class BlockLoader
{
  class BlockCreator;
public:
  typedef std::auto_ptr<BlockCreator> CreatorPtr;

  BlockLoader();

  BlockHandle
  get(const std::string & name);

  size_t
  load(const YAML::Node & node, std::ostream * const errStream);

  size_t
  load(const YAML::Node & node)
  {
    return load(node, &std::cerr);
  }

  void
  insert(const std::string & name, BlockHandle block);

  void
  registerCreator(const std::string & blockName, CreatorPtr creator);

private:
  typedef boost::ptr_map< const std::string, BlockCreator> BlocksDirectory;
  typedef std::map< std::string, BlockHandle> BlocksMap;
  typedef std::pair< std::string, std::string> TypeAndName;

  TypeAndName
  parseEntry(const std::string & entry) const;

  BlocksDirectory myBlocksDirectory;
  BlocksMap myBlocks;
  factory::BlockFactory myBlockFactory;
};

class BlockLoader::BlockCreator
{
public:
  virtual
  ~BlockCreator()
  {
  }
  virtual BlockHandle
  createBlock(const YAML::Node & node, std::ostream * const errStream) const = 0;
};

}
}

#endif /* BLOCK_LOADER_H */
