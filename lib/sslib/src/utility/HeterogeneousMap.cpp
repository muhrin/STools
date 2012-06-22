/*
 * HeterogeneousMap.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "utility/HeterogeneousMap.h"


// NAMESPACES ////////////////////////////////

namespace sstbx
{
namespace utility
{

bool HeterogeneousMap::empty() const
{
  return myAnyMap.empty();
}

size_t HeterogeneousMap::size() const
{
  return myAnyMap.size();
}

size_t HeterogeneousMap::max_size() const
{
  return myAnyMap.max_size();
}

void HeterogeneousMap::clear()
{
  myAnyMap.clear();
}

}
}
