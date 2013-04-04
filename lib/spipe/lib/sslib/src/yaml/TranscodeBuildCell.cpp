/*
 * TranscodeBuildCell.cpp
 *
 *  Created on: May 29, 2012
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "yaml/TranscodeBuildCell.h"

#include <boost/lexical_cast.hpp>

#include "factory/SsLibYamlKeywords.h"
#include "yaml/TranscodeGeneral.h"

// NAMESPACES ////////////////////////////////
namespace ssbc = ::sstbx::build_cell;

namespace YAML {

Node convert<ssbc::Sphere>::encode(const ssbc::Sphere & sphere)
{
  namespace kw = ::sstbx::factory::sslib_yaml_keywords;

  Node node;

  node[kw::POS] = sphere.getPosition();
  node[kw::RADIUS] = sphere.getRadius();

  return node;
}

bool convert<ssbc::Sphere>::decode(const Node & node, ssbc::Sphere & sphere)
{
  namespace kw = ::sstbx::factory::sslib_yaml_keywords;

  try
  {
    if(node[kw::POS])
      sphere.setPosition(node[kw::POS].as< ::arma::vec3>());

    if(node[kw::RADIUS])
      sphere.setRadius(node[kw::RADIUS].as<double>());

    if(node[kw::VOL])
      sphere.setRadius(ssbc::Sphere::radius(node[kw::VOL].as<double>()));
  }
  catch(const ::boost::bad_lexical_cast & /*exception*/)
  {
    return false;
  }

  return true;
}

}


