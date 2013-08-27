/*
 * TranscodePotential.h
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

#ifndef TRANSCODE_POTENTIAL_H
#define TRANSCODE_POTENTIAL_H

// INCLUDES //////////////////////////////////
#include "spl/SSLib.h"

#ifdef SSLIB_USE_YAML

#include <yaml-cpp/yaml.h>

#include "spl/potential/OptimisationSettings.h"
#include "spl/potential/CombiningRules.h"

// NAMESPACES ////////////////////////////////

// Some custom YAML transcoders
namespace YAML {

// potential

// potential::OptimisationSettings
template<>
struct convert< ::spl::potential::OptimisationSettings>
{
  static Node encode(const ::spl::potential::OptimisationSettings & rhs);
  static bool decode(const Node& node, ::spl::potential::OptimisationSettings & rhs);
};

// potential::SimplePairPotential::CombiningRule

// Simple pair potential combining rules
template<>
struct convert< ::spl::potential::CombiningRule::Value>
{
private:
  typedef ::spl::potential::CombiningRule Rule;
public:
  static Node encode(const Rule::Value & rhs);
  static bool decode(const Node & node, Rule::Value & rhs);
};

}

#endif /* SSLIB_USE_YAML */
#endif /* TRANSCODE_POTENTIAL_H */
