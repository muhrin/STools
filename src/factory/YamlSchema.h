/*
 * YamlSchema.h
 *
 *
 *  Created on: May 29, 2012
 *      Author: Martin Uhrin
 */

#ifndef STOOLS__FACTORY__YAML_SCHEMA_H
#define STOOLS__FACTORY__YAML_SCHEMA_H

// INCLUDES /////////////////////////////////////////////

#include <schemer/Schemer.h>

#include <spipe/factory/factory.h>

// DEFINES //////////////////////////////////////////////

namespace stools {
namespace factory {

///////////////////////////////////////////////////////////
// CUSTOM MAPS
///////////////////////////////////////////////////////////

struct PipeSettings
{
  spipe::factory::Engine engine;
};

SCHEMER_MAP(PipeSettingsSchema, PipeSettings)
{
  spipe::factory::Engine engineDefault;
  engineDefault.serialEngine = spipe::factory::SerialEngine();

  element("engine", &PipeSettings::engine)->defaultValue(engineDefault);
}

struct Build : public PipeSettings
{
  std::string rngSeed;
  boost::optional< spipe::factory::blocks::BuildStructures> buildStructures;
  boost::optional< spipe::factory::blocks::WriteStructures> writeStructures;
};

SCHEMER_MAP(BuildSchema, Build)
{
  extends< PipeSettingsSchema>();

  element("rngSeed", &Build::rngSeed)->defaultValue("time");
  element("buildStructures", &Build::buildStructures)->defaultValue(
      spipe::factory::blocks::BuildStructures());
  element("writeStructures", &Build::writeStructures)->defaultValue(
      spipe::factory::blocks::WriteStructures());
}

struct Search : public Build
{
  std::string castepExe;

  boost::optional< spipe::factory::blocks::RunPotentialParamsQueue> runPotParamsQueue;
  boost::optional< spipe::factory::blocks::SearchStoichiometries> searchStoichiometries;
  boost::optional< spipe::factory::blocks::SweepPotentialParams> sweepPotentialParams;
  boost::optional< std::string> loadStructures;
  boost::optional< spipe::factory::blocks::CutAndPaste> cutAndPaste;
  boost::optional< spipe::factory::blocks::GeomOptimise> preGeomOptimise;
  boost::optional< spipe::factory::blocks::GeomOptimise> geomOptimise;
  boost::optional< spipe::factory::blocks::RemoveDuplicates> removeDuplicates;
#ifdef SPL_WITH_CGAL
  boost::optional< spipe::factory::blocks::KeepStableCompositions> keepStableCompositions;
#endif
  boost::optional< spipe::factory::blocks::KeepTopN> keepTopN;
  boost::optional< spipe::factory::blocks::KeepWithinXPercent> keepWithinXPercent;
  boost::optional< spipe::factory::blocks::FindSymmetryGroup> findSymmetryGroup;
  boost::optional< spipe::factory::blocks::WriteStructures> writeStructures;
};

SCHEMER_MAP(SearchSchema, Search)
{
  extends< BuildSchema>();

  element("castep", &Search::castepExe)->defaultValue("castep");

  element("runPotentialParamsQueue", &Search::runPotParamsQueue);
  element("searchStoichiometries", &Search::searchStoichiometries);
  element("sweepPotentialParams", &Search::sweepPotentialParams);
  element("loadStructures", &Search::loadStructures);
  element("cutAndPaste", &Search::cutAndPaste);
  element("preGeomOptimise", &Search::preGeomOptimise);
  element("geomOptimise", &Search::geomOptimise);
  element("removeDuplicates", &Search::removeDuplicates);
#ifdef SPL_WITH_CGAL
  element("keepStableCompositions", &Search::keepStableCompositions);
#endif
  element("keepTopN", &Search::keepTopN);
  element("keepWithinXPercent", &Search::keepWithinXPercent);
  element("findSymmetryGroup", &Search::findSymmetryGroup);
  element("writeStructures", &Search::writeStructures);
}

}
}

#endif /* STOOLS__FACTORY__YAML_SCHEMA_H */

