/*
 * KeepStableCompositions.cpp
 *
 *  Created on: Nov 21, 2013
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "blocks/KeepStableCompositions.h"

#ifdef SSLIB_USE_CGAL

#include <boost/iterator/transform_iterator.hpp>
#include <boost/thread/lock_guard.hpp>

#include <spl/analysis/GnuplotConvexHullPlotter.h>
#include <spl/common/Structure.h>

#include <pipelib/pipelib.h>

// NAMESPACES ////////////////////////////////
namespace spla = ::spl::analysis;
namespace splc = ::spl::common;
namespace splu = ::spl::utility;

namespace spipe {
namespace blocks {

struct GetStructure : public ::std::unary_function< common::StructureData * const,
    splc::Structure * const>
{
  splc::Structure *
  operator()(common::StructureData * const structureData) const
  {
    return structureData->getStructure();
  }
};

KeepStableCompositions::KeepStableCompositions() :
    Block("Keep stable compositions"),
    myWriteHull(false)
{
}

KeepStableCompositions::KeepStableCompositions(const bool writeHull) :
    Block("Keep stable compositions"),
    myWriteHull(writeHull)
{
}

void
KeepStableCompositions::pipelineInitialised()
{
  if(myWriteHull)
  {
    spla::GnuplotConvexHullPlotter::Settings settings;
    settings.outputDir = workingDir();
    myOutputter.reset(new spla::GnuplotConvexHullPlotter(settings));
  }
}

void
KeepStableCompositions::in(common::StructureData * const data)
{
  if(!data->getStructure())
  {
    drop(data);
    return;
  }

#ifdef SP_ENABLE_THREAD_AWARE
  ::boost::lock_guard< ::boost::mutex> lock(myMutex);
#endif
  myStructureStore.insert(data);
  myConvexHullStructures.reset();
}

void
KeepStableCompositions::pipelineFinishing()
{
  myStructureStore.clear();
  myConvexHullStructures.reset();
}

size_t
KeepStableCompositions::release()
{
  if(myConvexHullStructures.get() && myConvexHullStructures->numStable() == 0)
    return 0;

  if(myOutputter.get())
    myOutputter->outputHull(myConvexHullStructures->getHull(),
        *myConvexHullStructures);

  size_t numStable = 0;
  StructureStore::iterator it;
  while(!myStructureStore.empty())
  {
    it = myStructureStore.begin();
    if(myConvexHullStructures->getStability(*(*it)->getStructure())
        == spla::ConvexHullStructures::Stability::STABLE)
    {
      out(*it);
      ++numStable;
    }
    else
      drop(*it);

    myStructureStore.erase(it);
  }

  return numStable;
}

bool
KeepStableCompositions::hasData() const
{
  typedef ::boost::transform_iterator< GetStructure, StructureStore::const_iterator> StructuresIterator;

  static const GetStructure GET_STRUCTURE = GetStructure();

  if(myStructureStore.empty())
    return false;

  StructuresIterator first(myStructureStore.begin(), GET_STRUCTURE);
  StructuresIterator end(myStructureStore.end(), GET_STRUCTURE);

  const spla::ConvexHullStructures::EndpointLabels endpoints =
      spla::ConvexHullStructures::generateEndpoints(first, end);

  if(endpoints.empty())
    return 0;

  myConvexHullStructures.reset(new spla::ConvexHullStructures(endpoints));
  myConvexHullStructures->insert(first, end);

  return myConvexHullStructures->numStable() != 0;
}

}
}

#endif // SSLIB_USE_CGAL
