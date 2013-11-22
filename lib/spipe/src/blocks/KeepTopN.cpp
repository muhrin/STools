/*
 * KeepTopN.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "blocks/KeepTopN.h"

#include <spl/common/Structure.h>

#include <pipelib/pipelib.h>

#include "common/StructureData.h"

// NAMESPACES ////////////////////////////////

namespace spipe {
namespace blocks {

namespace ssc = ::spl::common;
namespace structure_properties = ssc::structure_properties;

KeepTopN::KeepTopN(const size_t keepTopN) :
    Block("Keep top N"), myKeepTopN(keepTopN),
    myStructureProperty(structure_properties::general::ENTHALPY),
    myUsePerAtom(false)
{
}

KeepTopN::KeepTopN(const size_t keepTopN, const StructureProperty & property) :
    Block("Keep top N"), myKeepTopN(keepTopN),
    myStructureProperty(property),
    myUsePerAtom(false)
{
}

KeepTopN::KeepTopN(const size_t keepTopN, const StructureProperty & property,
    const bool usePerAtom) :
    Block("Keep top N"), myKeepTopN(keepTopN),
    myStructureProperty(property),
    myUsePerAtom(usePerAtom)
{
}

void
KeepTopN::in(common::StructureData * const data)
{
  const ssc::Structure * const structure = data->getStructure();
  const double * const value = structure->getProperty(myStructureProperty);

  // Let anything that doesn't have the property through
  if(value == NULL)
  {
    out(data);
    return;
  }

  double localValue = *value;
  if(myUsePerAtom)
    localValue /= static_cast<double>(structure->getNumAtoms());

  keep(data, localValue);
}

size_t
KeepTopN::release()
{
  const size_t numReleased = myStructures.size();
  BOOST_FOREACH(Structures::reference structurePair, myStructures)
  {
    out(structurePair.second);
  }
  myStructures.clear();
  return numReleased;
}

bool
KeepTopN::hasData() const
{
  return !myStructures.empty();
}

void
KeepTopN::keep(StructureDataType * const structure, const double energy)
{
#ifdef SP_ENABLE_THREAD_AWARE
  boost::lock_guard<boost::mutex> guard(myMutex);
#endif

  myStructures[energy] = structure;
  if(myStructures.size() > myKeepTopN)
  {
    // We have gone over by one, remove the last
    Structures::reverse_iterator last = myStructures.rbegin();
    drop(last->second);
    // Erasing using reverse iterator is a bit annoying
    // see: http://stackoverflow.com/questions/1830158/how-to-call-erase-with-a-reverse-iterator
    myStructures.erase((++last).base());
  }
}

}
}
