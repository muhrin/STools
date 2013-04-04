/*
 * TranscodeCommon.cpp
 *
 *  Created on: May 29, 2012
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "yaml/TranscodeCommon.h"

#include "io/IoFunctions.h"
#include "factory/FactoryFwd.h"
#include "factory/SsLibYamlKeywords.h"
#include "utility/IndexingEnums.h"
#include "yaml/TranscodeGeneral.h"

// NAMESPACES ////////////////////////////////
namespace ssc = ::sstbx::common;
namespace ssf = ::sstbx::factory;
namespace ssio = ::sstbx::io;
namespace ssu = ::sstbx::utility;
namespace ssy = ::sstbx::yaml;

namespace YAML {

Node convert< ::sstbx::common::UnitCell>::encode(const ::sstbx::common::UnitCell & cell)
{
  namespace kw = ssf::sslib_yaml_keywords;
  using namespace ssu::cell_params_enum;

  Node node;
  const double (&params)[6] = cell.getLatticeParams();
  ::std::stringstream ss;

  // Do first initially
  ssio::writeToStream(ss, params[A], 10);
  // Now rest
  for(size_t i = B; i <= GAMMA; ++i)
  {
    ss << " ";
    ssio::writeToStream(ss, params[i], 10);
  }

  node[kw::STRUCTURE__CELL__ABC] = ss.str();
  node[kw::STRUCTURE__CELL__VOL] = cell.getVolume();

  return node;
}

bool convert< ::sstbx::common::UnitCell>::decode(const Node & node, ::sstbx::common::UnitCell & cell)
{
  namespace kw = ssf::sslib_yaml_keywords;
  using namespace ssu::cell_params_enum;

  if(!node[kw::STRUCTURE__CELL__ABC])
    return false;

  typedef ssy::VectorAsString<double> DoublesVec;
  // Maybe it is a string separated by spaces
  DoublesVec doublesVec;

  try
  {
    doublesVec = node[kw::STRUCTURE__CELL__ABC].as<DoublesVec>();
  }
  catch(const YAML::TypedBadConversion<DoublesVec> & /*e*/)
  {
    return false;
  }

  if(doublesVec->size() != 6)
    return false; // Expect six parameters

  double params[6];
  for(size_t i = 0; i < 6; ++i)
    params[i] = (*doublesVec)[i];

  // Finally set the lattice parameters
  cell.setLatticeParams(params);

  return true;
}


}


