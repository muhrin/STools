/*
 * PipeBuilder.h
 *
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef PIPE_BUILDER_H
#define PIPE_BUILDER_H

// INCLUDES /////////////////////////////////////////////
#include "StructurePipe.h"

// Local includes
#include "common/PipeFactoryYaml.h"
#include "common/YamlKeywords.h"


// FORWARD DECLARATIONS ////////////////////////////////////
namespace YAML
{
class Node;
}


namespace spipe
{
namespace common
{

class PipeBuilder
{
public:

#ifdef SP_USE_YAML
  SpPipelineTyp * buildPipeFromYaml(const YAML::Node & node);
#endif // SP_USE_YAML
  

private:

#ifdef SP_USE_YAML
  /** A mapping from block ids to the instances */
  typedef ::std::map<
    ::spipe::common::yaml_keywords::IdTyp,
    SpPipeBlockTyp *>   BlocksMapYaml;

  PipeFactoryYaml     myFactoryYaml;
#endif // SP_USE_YAML

};


}
}

#endif /* PIPE_BUILDER_H */
