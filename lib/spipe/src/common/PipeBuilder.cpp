/*
 * PipeBuilder.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "common/PipeBuilder.h"

#include <boost/foreach.hpp>

#include <pipelib/IPipeline.h>

// Local includes
#include "common/StructureData.h"
#include "common/SharedData.h"

#include <yaml-cpp/yaml.h>

// NAMESPACES ////////////////////////////////

namespace spipe
{
namespace common
{

namespace kw = ::spipe::common::yaml_keywords;


SpPipelineTyp * PipeBuilder::buildPipeFromYaml(const YAML::Node & node)
{
  // First let's build up the blocks
  BlocksMapYaml blocksMap;

  ::spipe::SpStartBlockTyp * startBlock = NULL;

  if(node[kw::BLOCKS])
  {
    const YAML::Node & blocksNode = node[kw::BLOCKS];
    startBlock = myFactoryYaml.createStartBlock(blocksNode[0]);

    for(size_t i = 1; i < blocksNode.size(); ++i)
    {
      const YAML::Node & blockNode = blocksNode[i];

      // First try to create the block
      SpPipeBlockTyp * b = myFactoryYaml.createBlock(blockNode);

      if(b)
      {
        ::std::string id;
        if(blockNode[kw::ID])
        {
          id = blockNode[kw::ID].as< ::std::string>();
        }
        else if(blockNode[kw::TYPE])
        {
          id = blockNode[kw::TYPE].as< ::std::string>();
        }
        else
        {
          SP_ASSERT("Couldn't find block id");
        }
        blocksMap.insert(BlocksMapYaml::value_type(id, b));
      }
    }
  }


  SpPipelineTyp * pipe = NULL;
  // Now build the pipeline
  if(node[kw::PIPELINE])
  {
    const YAML::Node & pipeNode = node[kw::PIPELINE];

    pipe = myFactoryYaml.createPipeline(pipeNode);

    if(pipe)
    {
      if(pipeNode[kw::PIPELINE__SEQ])
      {
        const YAML::Node & seqNode = pipeNode[kw::PIPELINE__SEQ];

        SpBlockTyp * prevBlock = startBlock;

        // Do the first as a start block
        if(seqNode[0])
        {

          // TEMPORARY
          if(startBlock)
            pipe->setStartBlock(*startBlock);

          /*
          const YAML::Node & startNode = seqNode[0];
          if(startNode[kw::ID])
          {
            const BlocksMapYaml::const_iterator it = blocksMap.find(startNode[kw::ID].as< ::std::string>());
            if(it != blocksMap.end())
            {
              pipe->setStartBlock(*it->second);
              prevBlock = it->second;
            }
          }*/
        }

        SpPipeBlockTyp * currentBlock = NULL;
        for(size_t i = 1; i < seqNode.size(); ++i)
        {
          currentBlock = NULL;

          const YAML::Node & bNode = seqNode[i];
          if(bNode[kw::ID])
          {
            const BlocksMapYaml::const_iterator it = blocksMap.find(bNode[kw::ID].as< ::std::string>());
            if(it != blocksMap.end())
            {
              currentBlock = it->second;
              if(prevBlock)
              {
                pipe->connect(*prevBlock, *currentBlock);
              }
            }
          }
          prevBlock = currentBlock;
        }
      }
    }
  }


  return pipe;
}

}
}
