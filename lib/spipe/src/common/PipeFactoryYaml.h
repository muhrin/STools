///*
// * PipeFactoryYaml.h
// *
// *
// *  Created on: Aug 17, 2011
// *      Author: Martin Uhrin
// */
//
//#ifndef PIPE_FACTORY_YAML_H
//#define PIPE_FACTORY_YAML_H
//
//// INCLUDES /////////////////////////////////////////////
//#include "StructurePipe.h"
//
//#ifdef SP_USE_YAML
//
//#include <map>
//#include <memory>
//
//#include <boost/ptr_container/ptr_vector.hpp>
//
//
//#include <pipelib/DefaultBarrier.h>
//
//// From SSTbx
//#include <io/StructureReadWriteManager.h>
//
//// Local includes
//#include "SsLibFactoryYaml.h"
//#include "common/YamlKeywords.h"
//
//// FORWARD DECLARATIONS ////////////////////////////////////
//namespace YAML
//{
//class Node;
//}
//
//namespace spipe
//{
//namespace blocks
//{
//class NiggliReduction;
//class PotentialGo;
//class RandomStructure;
//class RemoveDuplicates;
//class WriteStructure;
//}
//}
//
//namespace spipe
//{
//namespace common
//{
//
//class PipeFactoryYaml
//{
//public:
//
//  PipeFactoryYaml();
//
//  SpPipelineTyp *      createPipeline(const YAML::Node & pipeNode);
//  SpPipeBlockTyp *    createBlock(const YAML::Node & blockNode);
//  SpStartBlockTyp *   createStartBlock(const YAML::Node & blockNode);
//
//private:
//
//  /** A mapping from block ids to the instances */
//  typedef ::std::map<
//    ::spipe::common::yaml_keywords::IdTyp,
//    SpBlock *>   BlocksMap;
//  
//
////  ::spipe::blocks::RandomStructure *  createBlockRandomCrystal(const YAML::Node & blockNode);
//  ::pipelib::DefaultBarrier< ::spipe::StructureDataTyp, ::spipe::SharedDataTyp> *
//    createBarrier(const YAML::Node & node);
//  ::spipe::blocks::RandomStructure *  createBlockRandomCrystal(const YAML::Node & blockNode);
//  ::spipe::blocks::RemoveDuplicates * createDropDuplicates(const YAML::Node & node);
//  ::spipe::blocks::NiggliReduction *  createNiggli(const YAML::Node & blockNode);
//  ::spipe::blocks::PotentialGo *      createGeomOptimise(const YAML::Node & node);
//  ::spipe::blocks::WriteStructure *   createWriteStructures(const YAML::Node & node);
//
//  ::sstbx::io::StructureReadWriteManager & getStructureReadWriteManager();
//
//
//  /**
//  /* Containers to hold the pipelines and blocks created by the factory.
//  /* The factory retains ownership and will cleanly release these under the
//  /* RAII idiom when the factory is destroyed.
//  /**/
//  ::boost::ptr_vector<SpPipelineTyp>   myPipelines;
//  ::boost::ptr_vector<SpBlock>     myBlocks;
//  ::boost::ptr_vector< ::pipelib::DefaultBarrier< ::spipe::StructureDataTyp, ::spipe::SharedDataTyp> >
//    myBarriers;
//
//  ::std::auto_ptr< ::sstbx::io::StructureReadWriteManager> myStructureReadWriteManager;
//
//  SsLibFactoryYaml                    mySsLibFactory;
//
//};
//
//
//}
//}
//
//#endif // SP_USE_YAML
//
//#endif /* PIPE_FACTORY_YAML_H */
//
