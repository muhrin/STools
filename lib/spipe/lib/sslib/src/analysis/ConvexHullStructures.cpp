/*
 * ConvexHullStructures.cpp
 *
 *  Created on: Nov 10, 2011
 *      Author: Martin Uhrin
 */

#include "analysis/ConvexHullStructures.h"

#ifdef SSLIB_USE_CGAL

# include <boost/iterator/iterator_facade.hpp>

namespace sstbx {
namespace analysis {
namespace detail {

class StableStructuresIterator : public ::boost::iterator_facade<
    StableStructuresIterator,
    const common::Structure *,
    ::boost::forward_traversal_tag,
     const common::Structure *
   >
{
public:
  StableStructuresIterator(): myHullStructures(NULL) {}

  explicit StableStructuresIterator(const ConvexHullStructures * hullStructures):
    myHullStructures(hullStructures)
  {
    myIt = myHullStructures->structuresBegin();
    goToNextStable();
  }

private:

  const common::Structure * dereference() const { return *myIt; }

  void increment() { goToNextStable(); }

  bool equal(const StableStructuresIterator & other) const
  {
    if(isAtEnd() && other.isAtEnd())
      return true;

    return myIt == other.myIt;
  }

  void goToNextStable()
  {
    // Find the next stable structure
    for(; myIt != myHullStructures->structuresEnd(); ++myIt)
    {
      if(myHullStructures->getStability(**myIt) == ConvexHullStructures::Stability::STABLE)
        break;
    }
  }

  bool isAtEnd() const
  {
    // If we don't have hull structures then we're an end iterator,
    // oterwise check if we're at the end of the hull structures
    return myHullStructures == NULL ? true : myIt == myHullStructures->structuresEnd();
  }

  const ConvexHullStructures * myHullStructures;
  ConvexHullStructures::StructuresIterator myIt;

  friend class boost::iterator_core_access;
};

} // namespace detail

ConvexHullStructures::ConvexHullStructures(const EndpointLabels & endpoints):
    myConvexHull(endpoints)
{}

ConvexHullStructures::ConvexHullStructures(const EndpointLabels & endpoints,
    utility::Key< double> & convexProperty):
    myConvexHull(endpoints, convexProperty)
{}

void
ConvexHullStructures::insertStructure(const common::Structure & structure)
{
  myStructures[&structure] = myConvexHull.addStructure(structure);
}

ConvexHullStructures::StructuresIterator ConvexHullStructures::structuresBegin() const
{
  return StructuresIterator(myStructures.begin());
}

ConvexHullStructures::StructuresIterator ConvexHullStructures::structuresEnd() const
{
  return StructuresIterator(myStructures.end());
}

ConvexHullStructures::StableStructuresIterator ConvexHullStructures::stableStructuresBegin() const
{
  return detail::StableStructuresIterator(this);
}

ConvexHullStructures::StableStructuresIterator ConvexHullStructures::stableStructuresEnd() const
{
  return StableStructuresIterator();
}

ConvexHullStructures::Stability::Value ConvexHullStructures::getStability(const common::Structure & structure) const
{
  const Structures::const_iterator it = myStructures.find(&structure);
  SSLIB_ASSERT(it != myStructures.end());

  if(it->second == -1)
    return Stability::NOT_HULL_STRUCTURE;
  else if(*myConvexHull.isStable(it->second))
    return Stability::STABLE;
  else
    return Stability::UNSTABLE;
}

OptionalDouble ConvexHullStructures::distanceToHull(const common::Structure & structure) const
{
  const Structures::const_iterator it = myStructures.find(&structure);
  if(it == myStructures.end())
    return myConvexHull.distanceToHull(structure);
  else
    return myConvexHull.distanceToHull(it->second); // It's faster to calculate if we know the id
}

}
}

#endif // SSLIB_USE_CGAL
