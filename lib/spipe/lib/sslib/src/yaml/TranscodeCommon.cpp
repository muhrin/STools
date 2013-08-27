/*
 * TranscodeCommon.cpp
 *
 *  Created on: May 29, 2012
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "spl/yaml/TranscodeCommon.h"

#include "spl/io/IoFunctions.h"
#include "spl/factory/FactoryFwd.h"
#include "spl/factory/SsLibYamlKeywords.h"
#include "spl/utility/IndexingEnums.h"
#include "spl/yaml/TranscodeGeneral.h"

// NAMESPACES ////////////////////////////////
namespace ssc = ::spl::common;
namespace ssf = ::spl::factory;
namespace ssio = ::spl::io;
namespace ssu = ::spl::utility;
namespace ssy = ::spl::yaml;

namespace YAML {

Node convert< ::spl::common::UnitCell>::encode(const ::spl::common::UnitCell & cell)
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

bool convert< ::spl::common::UnitCell>::decode(const Node & node, ::spl::common::UnitCell & cell)
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


