/*
 * ConvexHull.h
 *
 *  Created on: Jul 9, 2013
 *      Author: Martin Uhrin
 */

#ifndef CONVEX_HULL_H
#define CONVEX_HULL_H

// INCLUDES ////////////
#include "SSLib.h"

#ifdef SSLIB_USE_CGAL

#include <map>
#include <vector>
#include <string>
#include <utility>

#include <boost/noncopyable.hpp>
#include <boost/optional.hpp>
#include <boost/scoped_ptr.hpp>

#include <CGAL/Cartesian_d.h>
#include <CGAL/Convex_hull_d.h>
#include <CGAL/Fraction_traits.h>
//#if CGAL_USE_GMP && CGAL_USE_MPFR
//#  include <CGAL/Gmpfr.h>
//#else
#  include <CGAL/CORE_Expr.h>
//#endif

#include "analysis/AbsConvexHullGenerator.h"
#include "common/AtomsFormula.h"
#include "analysis/CgalCustomKernel.h"
#include "common/Structure.h"
#include "utility/HeterogeneousMapKey.h"

// DEFINITION ///////////////////////

namespace sstbx {

// FORWARD DECLARATIONS ///////

namespace analysis {

class HullEntry;

class ConvexHull : public AbsConvexHullGenerator, ::boost::noncopyable
{
public:
//#ifdef CGAL_USE_GMP
//  typedef CGAL::Quotient<CGAL::Gmpfr> RT;
//#else
  typedef CGAL::Quotient<CORE::Expr> RT;
//#endif
  //typedef CGAL::Cartesian_d<RT> HullTraits;
  typedef CgalCustomKernel<RT> HullTraits;
public:

  typedef CGAL::Convex_hull_d<HullTraits> Hull;
  typedef HullTraits::Point_d PointD;
  typedef HullTraits::Vector_d VectorD;
  typedef ::std::pair<common::AtomsFormula, PointD> Endpoint;
  typedef ::std::vector<common::AtomsFormula> EndpointLabels;
  typedef ::std::vector<Endpoint> Endpoints;
  typedef PointD::Id PointId;

  typedef Endpoints::const_iterator EndpointsConstIterator;

  class HullEntry
  {
  public:
    HullEntry(const common::AtomsFormula & composition, const HullTraits::FT value, const PointId id);

    const common::AtomsFormula & getComposition() const;
    const HullTraits::FT getValue() const;
    const PointId & getId() const;
    const ::boost::optional<PointD> & getPoint() const;
  private:
    void setPoint(const PointD & point);

    common::AtomsFormula myComposition;
    HullTraits::FT myValue;
    PointId myId;
    ::boost::optional<PointD> myPoint;

    //friend void ConvexHull::generateHull() const;
    friend class ConvexHull;
  };

  typedef ::std::vector<HullEntry>::const_iterator EntriesConstIterator;

  template <typename InputIterator>
  static EndpointLabels generateEndpoints(InputIterator first, InputIterator last);

  ConvexHull(const EndpointLabels & endpoints);
  ConvexHull(const EndpointLabels & endpoints, utility::Key<double> & convexProperty);
  template <typename InputIterator>
  ConvexHull(InputIterator first, InputIterator last);
  virtual ~ConvexHull() {}

  PointId addStructure(const common::Structure & structure);
  template <typename InputIterator>
  ::std::vector<PointId> addStructures(InputIterator first, InputIterator last);

  /**
   * Get the number of dimensions, includeing the convex property dimension.
   */
  int dims() const;

  const Hull * getHull() const;

  VectorD composition(const VectorD & vec) const;
  PointD composition(const PointD & point) const;

  EndpointsConstIterator endpointsBegin() const;
  EndpointsConstIterator endpointsEnd() const;

  EntriesConstIterator entriesBegin() const;
  EntriesConstIterator entriesEnd() const;

  ::boost::optional<bool> isStable(const PointD & point) const;

private:

  typedef ::std::map<common::AtomsFormula, HullTraits::FT> ChemicalPotentials;
  typedef ::std::vector<HullEntry> HullEntries;

  PointId generateEntry(const common::Structure & structure);
  void updateChemicalPotential(const common::AtomsFormula & endpointFormula, const HullTraits::FT value);
  PointD generateHullPoint(const HullEntry & entry) const;
  void generateHull() const;
  bool canGenerate() const;
  void initEndpoints(const EndpointLabels & labels);

  // IMPORTANT: Make sure myEndpoints the first member because this is relied on by one of
  // the constructors
  Endpoints myEndpoints;
  const utility::Key<double> myConvexProperty;
  ChemicalPotentials myChemicalPotentials;
  const int myHullDims;
  mutable HullEntries myEntries;
  mutable ::boost::scoped_ptr<Hull> myHull;
};

template <typename InputIterator>
ConvexHull::EndpointLabels ConvexHull::generateEndpoints(InputIterator first, InputIterator last)
{
  ::std::set< ::std::string> speciesSet;
  ::std::vector< ::std::string> species;
  EndpointLabels endpoints;
  for(InputIterator it = first; it != last; ++it)
  {
    it->getAtomSpecies(species);
    ::std::set< ::std::string> localSet(species.begin(), species.end());
    if(localSet.size() == 1)
      speciesSet.insert(*localSet.begin());
    endpoints.clear();
  }

  BOOST_FOREACH(const ::std::string & s, speciesSet)
    endpoints.push_back(common::AtomsFormula(s));
  return endpoints;
}

template <typename InputIterator>
ConvexHull::ConvexHull(InputIterator first, InputIterator last):
myConvexProperty(common::structure_properties::general::ENTHALPY),
myHullDims(last - first)
{
  initEndpoints(generateEndpoints(first, last));
  addStructures(first, last);
}

template <typename InputIterator>
::std::vector<ConvexHull::PointId> ConvexHull::addStructures(InputIterator first, InputIterator last)
{
  ::std::vector<PointId> ids;
  for(InputIterator it = first; it != last; ++it)
    ids.push_back(addStructure(*it));
  return ids;
}

}
}

#endif // SSLIB_USE_CGAL

#endif /* CONVEX_HULL_H */
