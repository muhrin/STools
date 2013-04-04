/*
 * TranscodePotential.h
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

#ifndef TRANSCODE_POTENTIAL_H
#define TRANSCODE_POTENTIAL_H

// INCLUDES //////////////////////////////////
#include "SSLib.h"

#ifdef SSLIB_USE_YAML

#include <yaml-cpp/yaml.h>

#include "potential/OptimisationSettings.h"
#include "potential/CombiningRules.h"

// NAMESPACES ////////////////////////////////

// Some custom YAML transcoders
namespace YAML {

// potential

// potential::OptimisationSettings
template<>
struct convert< ::sstbx::potential::OptimisationSettings>
{
  static Node encode(const ::sstbx::potential::OptimisationSettings & rhs);
  static bool decode(const Node& node, ::sstbx::potential::OptimisationSettings & rhs);
};

// potential::SimplePairPotential::CombiningRule

// Simple pair potential combining rules
template<>
struct convert< ::sstbx::potential::CombiningRule::Value>
{
private:
  typedef ::sstbx::potential::CombiningRule Rule;
public:
  static Node encode(const Rule::Value & rhs);
  static bool decode(const Node & node, Rule::Value & rhs);
};

}

#endif /* SSLIB_USE_YAML */
#endif /* TRANSCODE_POTENTIAL_H */
