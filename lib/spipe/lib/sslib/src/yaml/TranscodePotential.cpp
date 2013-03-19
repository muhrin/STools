/*
 * TranscodePotential.cpp
 *
 *  Created on: May 29, 2012
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "yaml/TranscodePotential.h"

#include "factory/SsLibYamlKeywords.h"

// NAMESPACES ////////////////////////////////

namespace ssp = ::sstbx::potential;

namespace YAML {


// potential

// potential::OptimisationSettings
Node convert<ssp::OptimisationSettings>::encode(
  const ssp::OptimisationSettings & rhs)
{
  namespace kw = ::sstbx::factory::sslib_yaml_keywords;
  Node node;
  // TODO


  return node;
}

bool convert<ssp::OptimisationSettings>::decode(
  const Node& node, ssp::OptimisationSettings & rhs)
{
  namespace kw = ::sstbx::factory::sslib_yaml_keywords;
  if(!node.IsMap())
    return false;

  // TODO: Finish

  if(node[kw::OPTIMISATION_SETTINGS__PRESSURE])
  {
    try
    {
      ::arma::mat33 pressure;
      pressure.diag().fill(node[kw::OPTIMISATION_SETTINGS__PRESSURE].as<double>());
      rhs.pressure.reset(pressure);
    }
    catch(const YAML::TypedBadConversion< ::sstbx::potential::OptimisationSettings> & /*e*/)
    { return false; }
  }

  return true;
}

// potential::SimplePairPotential::CombiningRule

// Simple pair potential combining rules
Node convert<ssp::CombiningRule::Value>::encode(
  const Rule::Value & rhs)
{
  Node node;
  node = ssp::getStringFromRule(rhs);  
  return node;
}

bool convert<ssp::CombiningRule::Value>::decode(
  const Node& node, Rule::Value & rhs)
{
  if(!node.IsScalar())
    return false;

  rhs = ssp::getRuleFromString(node.Scalar());
  if(rhs == Rule::UNKNOWN)
    return false;

  return true;
}

}


