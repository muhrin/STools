///*
// * PipeFactoryYaml.cpp
// *
// *  Created on: Aug 18, 2011
// *      Author: Martin Uhrin
// */
//
//// INCLUDES //////////////////////////////////
//#include "common/PipeFactoryYaml.h"
//
//#ifdef SP_USE_YAML
//
//#include <boost/shared_ptr.hpp>
//
//#include <yaml-cpp/yaml.h>
//
//
//#include <pipelib/SingleThreadedPipeline.h>
//
//// Local includes
//#include "blocks/NiggliReduction.h"
//#include "blocks/PotentialGo.h"
//#include "blocks/RandomStructure.h"
//#include "blocks/RemoveDuplicates.h"
//#include "blocks/WriteStructure.h"
//#include "common/SsLibYamlKeywords.h"
//#include "common/StructureData.h"
//#include "common/SharedData.h"
//
//// NAMESPACES ////////////////////////////////
//
//namespace spipe {
//namespace common {
//
//// Alias for accessing keywords namespace
//namespace kw = ::spipe::common::yaml_keywords;
//namespace sslibkw = ::spipe::common::sslib_yaml_keywords;
//namespace spb     = ::spipe::blocks;
//namespace ssbc    = ::sstbx::build_cell;
//namespace ssio    = ::sstbx::io;
//namespace sslibbc = ::sstbx::build_cell;
//namespace ssp     = ::sstbx::potential;
//
//PipeFactoryYaml::PipeFactoryYaml()
//{
//}
//
//SpPipelineTyp * PipeFactoryYaml::createPipeline(const YAML::Node & pipeNode)
//{
//
//  SpPipelineTyp * pipe = NULL;
//
//    if(pipeNode[kw::TYPE])
//    {
//      ::std::string value;    // String values buffer
//      value = pipeNode[kw::TYPE].as< ::std::string>();
//
//      if(value == kw::PIPELINE__SINGLE_THREADED)
//      {
//        pipe = new ::pipelib::SingleThreadedPipeline< ::spipe::StructureDataTyp, ::spipe::SharedDataTyp>();
//        myPipelines.push_back(pipe);
//      }
//      // NOTE: Add new pipe types here with "else if" statements
//    }
//
//  return pipe;
//}
//
//SpPipeBlockTyp * PipeFactoryYaml::createBlock(const YAML::Node & blockNode)
//{
//
//  SpPipeBlockTyp * block = NULL;
//
//  // Does the block have a type?
//  if(blockNode[kw::TYPE])
//  {
//    ::std::string sValue;    // String values buffer
//    sValue = blockNode[kw::TYPE].as< ::std::string>();
//
//    if(sValue == kw::BLOCKS__NIGGLI)
//    {
//      block = createNiggli(blockNode);
//    }
//    else if(sValue == kw::BLOCKS__GEOM_OPTIMISE)
//    {
//      block = createGeomOptimise(blockNode);
//    }
//    else if(sValue == kw::BLOCKS__WRITE_STRS)
//    {
//      block = createWriteStructures(blockNode);
//    }
//    else if(sValue == kw::BLOCKS__DROP_DUPLICATES)
//    {
//      block = createDropDuplicates(blockNode);
//    }
//    else if(sValue == kw::BLOCKS__BARRIER)
//    {
//      block = createBarrier(blockNode);
//    }
//  }
//
//  return block;
//}
//
//SpStartBlockTyp * PipeFactoryYaml::createStartBlock(const YAML::Node & blockNode)
//{
//
//  SpStartBlockTyp * block = NULL;
//
//  // Does the block have a type?
//  if(blockNode[kw::TYPE])
//  {
//    ::std::string sValue;    // String values buffer
//    sValue = blockNode[kw::TYPE].as< ::std::string>();
//
//    if(sValue == kw::BLOCKS__RANDOM_CRYSTAL)
//    {
//      block = createBlockRandomCrystal(blockNode);
//    }
//  }
//
//  return block;
//}
//
//::pipelib::DefaultBarrier< ::spipe::StructureDataTyp, ::spipe::SharedDataTyp> *
//PipeFactoryYaml::createBarrier(const YAML::Node & node)
//{
//  ::pipelib::DefaultBarrier< ::spipe::StructureDataTyp, ::spipe::SharedDataTyp> * barrier
//    = new ::pipelib::DefaultBarrier< ::spipe::StructureDataTyp, ::spipe::SharedDataTyp>();
//
//  myBarriers.push_back(barrier);
//
//  return barrier;
//}
//
//::spipe::blocks::RandomStructure *
//PipeFactoryYaml::createBlockRandomCrystal(const YAML::Node & blockNode)
//{
//  using ::std::string;
//
//  SP_ASSERT(blockNode[kw::TYPE] && 
//    blockNode[kw::TYPE].as<string>() == kw::BLOCKS__RANDOM_CRYSTAL);
//
//  //::spipe::blocks::RandomStructure * block = NULL;
//
//  // Things we need to construct a random crystal block
//  int nStrs = 0;
//  ssbc::StructureDescriptionPtr strDesc;
//
//  if(blockNode[kw::BLOCKS__RANDOM_CRYSTAL__N_STRS])
//  {
//    nStrs = blockNode[kw::BLOCKS__RANDOM_CRYSTAL__N_STRS].as<int>();
//  }
//  else
//  {
//    // TODO: Throw parsing exception
//  }
//
//
//  // Let's see if we can get the structure description
//  if(blockNode[sslibkw::STR_DESC])
//  {
//    strDesc = mySsLibFactory.createStructureGenerator(blockNode[sslibkw::STR_DESC]);
//  }
//  else
//  {
//    // TODO: Throw exception
//  }
//
//
//  return new ::spipe::blocks::RandomStructure(
//    *mySsLibFactory.createCrystalStructureGenerator(blockNode[sslibkw::STR_GENERATOR]),
//    nStrs,
//    blocks::RandomStructure::StructureDescPtr(mySsLibFactory.createStructureDescription(blockNode[sslibkw::STR_DESC]))
//  );
//}
//
//::spipe::blocks::RemoveDuplicates * PipeFactoryYaml::createDropDuplicates(const YAML::Node & node)
//{
//  return new spb::RemoveDuplicates(*mySsLibFactory.createStructureSet(node[sslibkw::STR_SET]));
//}
//
//::spipe::blocks::NiggliReduction * PipeFactoryYaml::createNiggli(const YAML::Node & blockNode)
//{
//  ::spipe::blocks::NiggliReduction * const niggli = new ::spipe::blocks::NiggliReduction();
//
//  myBlocks.push_back(niggli);
//
//  return niggli;
//}
//
//::spipe::blocks::PotentialGo * PipeFactoryYaml::createGeomOptimise(const YAML::Node & node)
//{
//  // First create the potential that we need
//
//  ::spipe::blocks::PotentialGo * goBlock = NULL;
//
//  if(node[kw::BLOCKS__GEOM_OPTIMISE__OPTIMISER])
//  {
//    ssp::IGeomOptimiser * const go = mySsLibFactory.createGeometryOptimiser(node[kw::BLOCKS__GEOM_OPTIMISE__OPTIMISER]);
//
//    if(go)
//    {
//      goBlock = new ::spipe::blocks::PotentialGo(*go);
//      myBlocks.push_back(goBlock);
//    }
//  }
//
//  return goBlock;
//}
//
//::spipe::blocks::WriteStructure *
//PipeFactoryYaml::createWriteStructures(const YAML::Node & node)
//{
//  spb::WriteStructure * writeStrs = NULL;
//  if(node[sslibkw::STR_WRITER])
//  {
//    ssio::IStructureWriter * const writer = mySsLibFactory.createStructureWriter(node[sslibkw::STR_WRITER]);
//    ssio::StructureWriterManager & manager = getStructureWriterManager();
//
//    manager.registerWriter(*writer);
//
//    writeStrs = new spb::WriteStructure(manager);
//  }
//
//  if(writeStrs != NULL)
//    myBlocks.push_back(writeStrs);
//
//  return writeStrs;
//}
//
//ssio::StructureWriterManager & PipeFactoryYaml::getStructureWriterManager()
//{
//  if(myStructureWriterManager.get() == NULL)
//  {
//    myStructureWriterManager.reset(new ssio::StructureWriterManager());
//  }
//  return *myStructureWriterManager;
//}
//
//}
//}
//
//
//#endif // SP_USE_YAML
