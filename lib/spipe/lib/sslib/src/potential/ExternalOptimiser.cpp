/*
 * ExternalOptimiser.cpp
 *
 *  Created on: Jan 14, 2014
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "spl/potential/ExternalOptimiser.h"

#ifdef SPL_WITH_YAML

#include <fstream>

#include <yaml-cpp/yaml.h>

#include <schemer/Schemer.h>

#include <spl/os/Process.h>
#include <spl/potential/OptimisationSettings.h>
#include <spl/utility/UtilFunctions.h>
#include <spl/yaml/TranscodeGeneral.h>

// NAMESPACES ////////////////////////////////

namespace spl {
namespace potential {

SCHEMER_ENUM(OptimiseType, OptimisationSettings::Optimise::Value)
{
  enumeration("atoms", OptimisationSettings::Optimise::ATOMS);
  enumeration("lattice", OptimisationSettings::Optimise::LATTICE);
  enumeration("atomsAndLattice",
      OptimisationSettings::Optimise::ATOMS_AND_LATTICE);
}

struct OptSettings
{
  boost::optional< int> maxIter;
  boost::optional< double> energyTol;
  boost::optional< double> forceTol;
  boost::optional< double> stressTol;
  boost::optional< arma::mat33> pressure;
  boost::optional< OptimisationSettings::Optimise::Value> optimisationType;
};

SCHEMER_MAP(OptSettingsType, OptSettings)
{
  element("maxIter", &OptSettings::maxIter);
  element("energyTol", &OptSettings::energyTol);
  element("ForceTol", &OptSettings::forceTol);
  element("stressTol", &OptSettings::stressTol);
  element< schemer::Scalar< arma::mat33> >("pressure", &OptSettings::pressure);
  element("optimise", &OptSettings::optimisationType);
}

struct OptResults
{
  bool successful;
  boost::optional< int> finalIters;
  boost::optional< double> finalEnthalpy;
  boost::optional< double> finalInternalEnergy;
  boost::optional< double> finalPressure;
};

SCHEMER_MAP(OptResultsType, OptResults)
{
  element("successful", &OptResults::successful);
  element("finalIters", &OptResults::finalIters);
  element("finalEnthalpy", &OptResults::finalEnthalpy);
  element("finalInternalEnergy", &OptResults::finalInternalEnergy);
  element("finalPressure", &OptResults::finalPressure);
}

ExternalOptimiser::ExternalOptimiser(const std::string & runCommand) :
    myRunCommand(runCommand)
{
}

OptimisationOutcome
ExternalOptimiser::optimise(common::Structure & structure,
    const OptimisationSettings & options) const
{
  OptimisationData data;
  return optimise(structure, data, options);
}

OptimisationOutcome
ExternalOptimiser::optimise(common::Structure & structure,
    OptimisationData & data, const OptimisationSettings & options) const
{
  const std::string outputStem = utility::generateUniqueName(10);
  const std::string settingsFile = outputStem + ".yaml";
  const std::string strFile = outputStem + ".res";

  // Write the settings and the structure file
  writeSettings(settingsFile, options);
  myResReaderWriter.writeStructure(structure, io::ResourceLocator(strFile));

  // Set up cargs to process: [options file] [structure file]
  std::vector< std::string> args;
  args.push_back(settingsFile);
  args.push_back(strFile);

  if(os::runBlocking(myRunCommand, args) != 0)
    return OptimisationOutcome::failure(OptimisationError::INTERNAL_ERROR);

  OptResults results;
  if(!readResults(settingsFile, &results) || !results.successful)
    return OptimisationOutcome::failure(OptimisationError::INTERNAL_ERROR);

  data.internalEnergy = results.finalInternalEnergy;
  data.enthalpy = results.finalEnthalpy;
  data.numIters = results.finalIters;
  data.pressure = results.finalPressure;

  // Copy over the new structure
  common::StructurePtr newStructure = myResReaderWriter.readStructure(
      io::ResourceLocator(strFile));
  structure = *newStructure;

  return OptimisationOutcome::success();
}

void
ExternalOptimiser::writeSettings(const std::string & filename,
    const OptimisationSettings & settings) const
{
  if(filename.empty())
    return;

  OptSettings optSettings;
  optSettings.maxIter = settings.maxIter;
  optSettings.energyTol = settings.energyTol;
  optSettings.forceTol = settings.forceTol;
  optSettings.stressTol = settings.stressTol;
  optSettings.pressure = settings.pressure;
  optSettings.optimisationType = settings.optimisationType;

  YAML::Node settingsNode;
  schemer::serialise(optSettings, &settingsNode);

  std::ofstream settingsFile(filename.c_str());
  if(settingsFile.is_open())
  {
    settingsFile << settingsNode;
    settingsFile.close();
  }
}

bool
ExternalOptimiser::readResults(const std::string & filename,
    OptResults * const results) const
{
  if(filename.empty())
    return false;

  // Read the results in YAML
  YAML::Node resultsNode;
  try
  {
    resultsNode = YAML::LoadFile(filename);
  }
  catch(const YAML::Exception & /*e*/)
  {
    return false;
  }

  if(!schemer::parse(resultsNode, results))
    return false;

  return true;
}

}
}

#endif // SSLIB_WITH_YAML
