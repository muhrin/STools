/*
 * ConvexHullStructures.h
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef STABLE_COMPOSITIONS_H
#define STABLE_COMPOSITIONS_H

// INCLUDES ////////////
#include "SSLib.h"

#ifdef SSLIB_USE_CGAL

#include <map>

#include <boost/iterator/transform_iterator.hpp>

#include "analysis/ConvexHull.h"
#include "utility/HeterogeneousMapKey.h"
#include "utility/TransformFunctions.h"


// DEFINITION ///////////////////////

namespace sstbx {

// FORWARD DECLARATIONS ///////
namespace common {
  class Structure;
}

namespace analysis {
namespace detail {
class StableStructuresIterator;
}

class ConvexHullStructures
{
  typedef ::std::map<common::Structure *, ConvexHull::PointId> Structures;
  typedef utility::TakeFirst<const Structures::value_type> TakeFirst;
public:

  typedef ConvexHull::EndpointLabels EndpointLabels;
  typedef ::boost::transform_iterator<TakeFirst, Structures::const_iterator> StructuresIterator;
  typedef detail::StableStructuresIterator StableStructuresIterator;

  struct Stability
  {
    enum Value { UNSTABLE, STABLE, NOT_HULL_STRUCTURE };
  };

  template< typename InputIterator>
    static EndpointLabels
    generateEndpoints(InputIterator first, InputIterator last);

  ConvexHullStructures(const EndpointLabels & endpoints);
  ConvexHullStructures(const EndpointLabels & endpoints,
      utility::Key< double> & convexProperty);
  template< typename InputIterator>
  ConvexHullStructures(InputIterator first, InputIterator last);

  StructuresIterator
  insertStructure(common::Structure & structure);
  template< typename InputIterator>
    void
    insertStructures(InputIterator first, InputIterator last);

  StructuresIterator structuresBegin() const;
  StructuresIterator structuresEnd() const;

  StableStructuresIterator stableStructuresBegin() const;
  StableStructuresIterator stableStructuresEnd() const;

  Stability::Value getStability(const common::Structure & structure) const;

  double getFormationEnthalpy(const StructuresIterator structure) const;
  void populateFormationEnthalpies();

  OptionalDouble distanceToHull(const common::Structure & structure) const;
private:

  double getFormationEnthalpy(const ConvexHull::PointId id) const;

  ConvexHull myConvexHull;
  Structures myStructures;
};

template< typename InputIterator>
ConvexHullStructures::ConvexHullStructures(InputIterator first, InputIterator last):
myConvexHull(ConvexHull::generateEndpoints(first, last))
{
  insertStructures(first, last);
}

template< typename InputIterator>
  void
  ConvexHullStructures::insertStructures(InputIterator first, InputIterator last)
{
  for(InputIterator it = first; it != last; ++it)
    insertStructure(*it);
}

template< typename InputIterator>
  ConvexHullStructures::EndpointLabels
  ConvexHullStructures::generateEndpoints(InputIterator first, InputIterator last)
{
  return ConvexHull::generateEndpoints(first, last);
}

}
}

#include "analysis/detail/ConvexHullStructures.h"

#endif // SSLIB_USE_CGAL
#endif /* STABLE_COMPOSITIONS_H */
