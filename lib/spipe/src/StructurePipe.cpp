/*
 * StructurePipe.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include <StructurePipe.h>

#include <yaml-cpp/yaml.h>

#include <pipelib/pipelib.h>

// Local includes
#include <spl/common/SharedData.h>
#include <spl/common/StructureData.h>
#include <spl/common/PipeBuilder.h>
#include <spl/common/PipeFactoryYaml.h>

// TODO: TEMPORARY!!
#include <spl/factory/ISchemaElementInstance.h>
#include <spl/factory/SsLibElements.h>
#include <spl/factory/SchemaDoc.h>
#include <spl/factory/SchemaMap.h>
#include <spl/common/YamlInputObjectAdapter.h>

// MACROS ////////////////////////////////////

// NAMESPACES ////////////////////////////////


int main(const int argc, const char * const argv[])
{
/*
  {// START -- TEMPORARY //////////////////////

  namespace ssf = ::spl::factory;

  ssf::SsLibElements lib;

  ssf::SchemaMap root("root", false);

  root.insert(lib.cellDesc->newInstance());

  ssf::SchemaDoc doc(root.newInstance());
  ::spipe::common::YamlInputObjectAdapter testNode(YAML::LoadFile("test.spipe"), "ROOT");
  doc.getRoot()->validate(testNode, doc);

  return 0;

  }// END -- TEMPORARY /////////////////////
*/

  //namespace sp = ::spipe;
  //namespace spcom = ::spipe::common;

  //if(argc != 2)
  //{
  //  ::std::cout << "Usage: " << argv[0] << " [yaml pipline in file]";
  //  return 1;
  //}

  //try
  //{
  //  YAML::Node config = YAML::LoadFile(argv[1]);

  //  spcom::PipeBuilder builder;

  //  sp::SpPipelineTyp * pipe = builder.buildPipeFromYaml(config);

  //  if(pipe)
  //  {
  //    pipe->initialise();
  //    pipe->start();
  //  }
  //}
  //catch(const YAML::Exception & e)
  //{
  //  ::std::cerr << "Error parsing YAML file\n" << e.what();
  //}

	return 0;
}

