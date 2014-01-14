/*
 * BlockLoader.cpp
 *
 *  Created on: Dec 13, 2013
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "build/BlockLoader.h"

#include <sstream>

#include <boost/foreach.hpp>

#include "factory/YamlSchema.h"

// NAMESPACES ////////////////////////////////

namespace spipe {
namespace build {

template< typename T>
  class FactoryBlockCreator : public BlockLoader::BlockCreator
  {
  public:
    FactoryBlockCreator(const factory::BlockFactory & factory) :
        myFactory(factory)
    {
    }
    virtual
    ~FactoryBlockCreator()
    {
    }
    virtual BlockHandle
    createBlock(const YAML::Node & node, std::ostream * const errStream) const
    {
      static const T SCHEMA_TYPE = T();
      typename T::BindingType factoryType;
      schemer::ParseLog log;
      SCHEMA_TYPE.nodeToValue(node, &factoryType, &log);
      if(log.hasErrors())
        log.printErrors();

      BlockHandle handle;
      myFactory.createBlock(&handle, factoryType);
      return handle;
    }
  private:
    const factory::BlockFactory & myFactory;
  };

BlockLoader::BlockLoader()
{
  registerCreator("buildStructures",
      CreatorPtr(
          new FactoryBlockCreator< factory::blocks::BuildStructuresSchema>(
              myBlockFactory)));
  registerCreator("clone",
      CreatorPtr(
          new FactoryBlockCreator< factory::blocks::CloneSchema>(
              myBlockFactory)));
  registerCreator("cutAndPaste",
      CreatorPtr(
          new FactoryBlockCreator< factory::blocks::CutAndPasteSchema>(
              myBlockFactory)));
  registerCreator("findSymmetry",
      CreatorPtr(
          new FactoryBlockCreator< factory::blocks::FindSymmetryGroupSchema>(
              myBlockFactory)));
  registerCreator("keepStableCompositions",
      CreatorPtr(
          new FactoryBlockCreator< factory::blocks::KeepStableCompositionsSchema>(
              myBlockFactory)));
  registerCreator("keepTopN",
      CreatorPtr(
          new FactoryBlockCreator< factory::blocks::KeepTopNSchema>(
              myBlockFactory)));
  registerCreator("keepWithinXPercent",
      CreatorPtr(
          new FactoryBlockCreator< factory::blocks::KeepWithinXPercentSchema>(
              myBlockFactory)));
  registerCreator("loadStructures",
      CreatorPtr(
          new FactoryBlockCreator< factory::blocks::LoadStructures>(
              myBlockFactory)));
  registerCreator("niggliReduce",
      CreatorPtr(
          new FactoryBlockCreator< factory::blocks::NiggliReduceSchema>(
              myBlockFactory)));
  registerCreator("geomOptimise",
      CreatorPtr(
          new FactoryBlockCreator< factory::blocks::GeomOptimiseSchema>(
              myBlockFactory)));
  registerCreator("removeDuplicates",
      CreatorPtr(
          new FactoryBlockCreator< factory::blocks::RemoveDuplicatesSchema>(
              myBlockFactory)));
  registerCreator("runPotentialParamsQueue",
      CreatorPtr(
          new FactoryBlockCreator< factory::blocks::RunPotentialParamsQueueSchema>(
              myBlockFactory)));
  registerCreator("searchStoichiometries",
      CreatorPtr(
          new FactoryBlockCreator< factory::blocks::SearchStoichiometriesSchema>(
              myBlockFactory)));
  registerCreator("sweepPotentialParams",
      CreatorPtr(
          new FactoryBlockCreator< factory::blocks::SweepPotentialParamsSchema>(
              myBlockFactory)));
  registerCreator("writeStructures",
      CreatorPtr(
          new FactoryBlockCreator< factory::blocks::WriteStructuresSchema>(
              myBlockFactory)));
}

BlockHandle
BlockLoader::get(const ::std::string & name)
{
  const BlocksMap::const_iterator it = myBlocks.find(name);
  if(it == myBlocks.end())
    return BlockHandle();

  return it->second;
}

size_t
BlockLoader::load(const YAML::Node & node, std::ostream * const errStream)
{
  using YAML::Node;

  // Format for map entries that are blocks is:
  //
  // blockType [name]: [block info]
  //
  // if [name] is not present then the name is assumed to be
  // whatever blockType is

  if(!node.IsMap())
  {
    if(errStream)
      *errStream << "Error, YAML node supplied to load blocks is not a map.\n";
    return 0;
  }

  size_t numLoaded = 0;
  BlocksDirectory::const_iterator creator;
  std::string temp;
  for(Node::const_iterator it = node.begin(), end = node.end(); it != end; ++it)
  {
    // Find out what type of block to create
    std::stringstream ss(it->first.Scalar());
    ss >> temp;
    creator = myBlocksDirectory.find(temp);
    if(creator != myBlocksDirectory.end())
    {
      // Find out the name (if present)
      if(ss.good())
        ss >> temp;

      // Try to create the block
      BlockHandle handle = creator->second->createBlock(it->second, errStream);
      if(handle)
      {
        insert(temp, handle);
        ++numLoaded;
      }
    }
  }
  return numLoaded;
}

void
BlockLoader::insert(const std::string & name, BlockHandle block)
{
  myBlocks[name] = block;
}

void
BlockLoader::registerCreator(const std::string & blockName, CreatorPtr creator)
{
  myBlocksDirectory.insert(blockName, creator);
}

}
}
