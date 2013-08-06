/*
 * ConvexHull.cpp
 *
 *  Created on: Nov 10, 2011
 *      Author: Martin Uhrin
 */

#include "analysis/ConvexHull.h"

#ifdef SSLIB_USE_CGAL

#include <list>

#include <boost/foreach.hpp>

#include <CGAL/Delaunay_d.h>

#include "common/StructureProperties.h"

//#define DEBUG_CONVEX_HULL_GENERATOR

namespace sstbx {
namespace analysis {

ConvexHull::ConvexHull(const EndpointLabels & labels) :
    myConvexProperty(common::structure_properties::general::ENTHALPY), myHullDims(
        labels.size())
{
  SSLIB_ASSERT_MSG(labels.size() >= 2,
      "Need at least two endpoints to make convex hull.");
  initEndpoints(labels);
}

ConvexHull::ConvexHull(const EndpointLabels & labels,
    utility::Key< double> & convexProperty) :
    myConvexProperty(convexProperty), myHullDims(labels.size())
{
  SSLIB_ASSERT_MSG(labels.size() >= 2,
      "Need at least two endpoints to make convex hull.");
  initEndpoints(labels);
}

const ConvexHull::HullTraits::FT ConvexHull::FT_ZERO(0.0);
const ConvexHull::HullTraits::RT ConvexHull::RT_ZERO(0.0);

ConvexHull::PointId
ConvexHull::addStructure(const common::Structure & structure)
{
  return generateEntry(structure);
}

int
ConvexHull::dims() const
{
  return myHullDims;
}

const ConvexHull::Hull *
ConvexHull::getHull() const
{
  if(!myHull.get() && canGenerate())
  {
    generateHull();
#ifdef DEBUG_CONVEX_HULL_GENERATOR
    myHull->print_statistics();
#endif
  }

  return myHull.get();
}

ConvexHull::PointD
ConvexHull::composition(const PointD & point) const
{
  return PointD(myHullDims - 1, point.cartesian_begin(),
      point.cartesian_end() - 1);
}

ConvexHull::EndpointsConstIterator
ConvexHull::endpointsBegin() const
{
  return myEndpoints.begin();
}

ConvexHull::EndpointsConstIterator
ConvexHull::endpointsEnd() const
{
  return myEndpoints.end();
}

ConvexHull::EntriesConstIterator
ConvexHull::entriesBegin() const
{
  return myEntries.begin();
}

ConvexHull::EntriesConstIterator
ConvexHull::entriesEnd() const
{
  return myEntries.end();
}

::boost::optional< bool>
ConvexHull::isStable(const PointD & point) const
{
  if(!myHull.get())
    return ::boost::optional< bool>();

  return point[dims() - 1] <= 0
      && myHull->bounded_side(point) == CGAL::ON_BOUNDARY;
}

::boost::optional<bool>
ConvexHull::isStable(const PointId id) const
{
  if(!myEntries[id].getPoint())
    return ::boost::optional< bool>();

  return isStable(*myEntries[id].getPoint());
}

OptionalDouble ConvexHull::distanceToHull(const common::Structure & structure) const
{
  // Check if the structure has a value for the property that will form the 'depth' of the hull
  const double * const value = structure.getProperty(myConvexProperty);
  if(!value)
    return OptionalDouble();

  const PointD p = generateHullPoint(structure.getComposition(), HullTraits::RT(*value));

  OptionalDouble dist;
  const ::boost::optional<HullTraits::RT> ftDist = distanceToHull(p);
  if(ftDist)
    dist.reset(CGAL::to_double(*ftDist));
  return dist;
}

OptionalDouble ConvexHull::distanceToHull(const PointId id) const
{
  OptionalDouble dist;
  const ::boost::optional<HullTraits::RT> ftDist = distanceToHull(*myEntries[id].getPoint());
  if(ftDist)
    dist.reset(CGAL::to_double(*ftDist));
  return dist;
}

ConvexHull::HullEntry::HullEntry(const common::AtomsFormula & composition,
    const HullTraits::FT value, const ConvexHull::PointId id,
    const bool isEndpoint) :
    myComposition(composition), myValue(value), myId(id), myIsEndpoint(
        isEndpoint)
{
}

const common::AtomsFormula &
ConvexHull::HullEntry::getComposition() const
{
  return myComposition;
}

const ConvexHull::HullTraits::FT
ConvexHull::HullEntry::getValue() const
{
  return myValue;
}

const ConvexHull::PointId &
ConvexHull::HullEntry::getId() const
{
  return myId;
}

const ::boost::optional< ConvexHull::PointD> &
ConvexHull::HullEntry::getPoint() const
{
  return myPoint;
}

bool
ConvexHull::HullEntry::isEndpoint() const
{
  return myIsEndpoint;
}

void
ConvexHull::HullEntry::setPoint(const PointD & p)
{
  myPoint = p;
}

ConvexHull::PointId
ConvexHull::generateEntry(const common::Structure & structure)
{
  // Check if the structure has a value for the property that will form the 'depth' of the hull
  const double * const value = structure.getProperty(myConvexProperty);
  if(!value)
    return -1;

  // Check that the structure contains at least one endpoint
  common::AtomsFormula composition = structure.getComposition();
  bool containsEndpoint = false, isEndpoint = true;
  int endpointFormulaUnits;
  common::AtomsFormula endpoint;
  ::std::pair< int, int> endpointUnits;
  BOOST_FOREACH(Endpoints::const_reference e, myEndpoints)
  {
    endpointUnits = composition.numberOf(e.first);

    if(endpointUnits.first != 0)
    {
      // Do a quick sanity check
      SSLIB_ASSERT(endpointUnits.second == 1);

      if(containsEndpoint)
        isEndpoint = false; // Contains more than one endpoint
      else
      {
        containsEndpoint = true; // The first endpoint we've come across so far
        endpointFormulaUnits = endpointUnits.first;
        endpoint = e.first;
      }
      composition.remove(e.first, endpointUnits.first);
    }
  }
  if(!containsEndpoint)
    return -1; // Doesn't contain any endpoints so it's nowhere in relation to hull
  if(!composition.isEmpty())
    return -1; // This structure has atoms that aren't on the hull so we can't calculate the chemical potential

  const PointId id = myEntries.size();
  myEntries.insert(myEntries.end(), HullEntry(structure.getComposition(), *value, id, isEndpoint));
  if(isEndpoint)
    updateChemicalPotential(endpoint,
        HullTraits::FT(*value, endpointFormulaUnits));

  return id;
}

void
ConvexHull::updateChemicalPotential(
    const common::AtomsFormula & endpointFormula, const HullTraits::FT value)
{
  ChemicalPotentials::iterator it = myChemicalPotentials.find(endpointFormula);
  if(it == myChemicalPotentials.end())
    myChemicalPotentials[endpointFormula] = value;
  else
  {
    if(it->second > value)
    {
#ifdef DEBUG_CONVEX_HULL_GENERATOR
      ::std::cout << "Updating endpoint " << endpointFormula << " to " << value << ::std::endl;
#endif
      it->second = value;
      myHull.reset(); // Hull needs to be re-generated
    }
  }
}

ConvexHull::PointD
ConvexHull::generateHullPoint(const HullEntry & entry) const
{
  PointD point = generateHullPoint(entry.getComposition(), entry.getValue());
  point.setId(entry.getId());
  return point;
}

ConvexHull::PointD
ConvexHull::generateHullPoint(const common::AtomsFormula & composition, const HullTraits::FT & convexValue) const
{
  // Need chemical potentials for all endpoints
  SSLIB_ASSERT(canGenerate());

  int totalAtoms = 0;
  HullTraits::FT totalMuNAtoms = FT_ZERO;
  int numAtoms;
  BOOST_FOREACH(Endpoints::const_reference endpoint, myEndpoints)
  {
    const ::std::pair< int, int> & frac = composition.numberOf(endpoint.first);
    numAtoms = frac.first;
    if(numAtoms != 0)
    {
      // Do a quick sanity check
      SSLIB_ASSERT(frac.second == 1);

      totalAtoms += numAtoms;
      totalMuNAtoms += myChemicalPotentials.find(endpoint.first)->second
          * HullTraits::FT(numAtoms);
    }
  }

  // Create a vector that is the weighted sum of the vectors of composition
  // simplex
  ::std::vector< HullTraits::FT> tempVec(myHullDims);
  VectorD v(myHullDims);
  int numEndpoints = 0;
  for(int i = 0; i < myEndpoints.size(); ++i)
  {
    numAtoms = composition.numberOf(myEndpoints[i].first).first;
    if(numAtoms != 0)
    {
      VectorD scaled = myEndpoints[i].second - CGAL::ORIGIN;
#ifdef DEBUG_CONVEX_HULL_GENERATOR
      ::std::cout << "Adding weighted hull endpoint: " << scaled << " weight: " << HullTraits::FT(numAtoms, totalAtoms) << ::std::endl;
#endif
      scaled *= HullTraits::FT(numAtoms, totalAtoms);
      v += scaled;
      ++numEndpoints;
    }
  }
  tempVec.assign(v.cartesian_begin(), v.cartesian_end());
  // The last hull coordinate is always the 'convex property', usually the energy
  if(numEndpoints > 1)
    tempVec[dims() - 1] = (convexValue - totalMuNAtoms) / totalAtoms;

  PointD point(myHullDims, tempVec.begin(), tempVec.end());

#ifdef DEBUG_CONVEX_HULL_GENERATOR
  ::std::cout << "Convex value: " << convexValue << " mu_alpha N_alpha: " << totalMuNAtoms << " total atoms: " << totalAtoms << ::std::endl;
  ::std::cout << "Point entry: " << point << ::std::endl;
#endif

  return point;
}

void
ConvexHull::generateHull() const
{
  SSLIB_ASSERT(canGenerate());

  typedef ::std::map< common::AtomsFormula, HullEntry *> LowestEnergy;
  typedef ::std::map< HullTraits::FT, HullEntry *> SortedEntries;

  // To make calculating the hull faster and remove redundant points
  // first get the set of points with the lowest energy at composition coordinate
  LowestEnergy lowest;
  LowestEnergy::iterator it;
  BOOST_FOREACH(HullEntries::reference entry, myEntries)
  {
    const PointD & p = generateHullPoint(entry);
    entry.setPoint(p);

    if(p[dims() - 1] <= 0)
    {
      common::AtomsFormula composition = entry.getComposition();
      composition.reduce();
      it = lowest.find(composition);
      if(it == lowest.end())
        lowest[composition] = &entry;
      else
      {
        // If this point is lower in energy then replace the old one
        if(p[dims() - 1] < (*it->second->getPoint())[dims() - 1])
          it->second = &entry;
      }
    }
  }

  // Now sort the remaining entries by energy
  SortedEntries sortedEntries;
  BOOST_FOREACH(LowestEnergy::reference entry, lowest)
  {
    sortedEntries[(*entry.second->getPoint())[dims() - 1]] = entry.second;
  }

  myHull.reset(new Hull(myHullDims));
  // Put in the endpoints first
  BOOST_FOREACH(Endpoints::const_reference ep, myEndpoints)
  {
    myHull->insert(ep.second);
  }
  // Now put the points in into the hull from lowest up
  BOOST_FOREACH(SortedEntries::const_reference entry, sortedEntries)
  {
    if(!entry.second->isEndpoint())
      myHull->insert(*entry.second->getPoint());
  }
}

bool
ConvexHull::canGenerate() const
{
  return myChemicalPotentials.size() == myEndpoints.size();
}

void
ConvexHull::initEndpoints(const EndpointLabels & labels)
{
  for(int i = 0; i < labels.size(); ++i)
  {
    myEndpoints.push_back(::std::make_pair(labels[i], PointD(myHullDims)));
    // Make sure we're using the reduced, i.e. most simple, formula
    // e.g. A4B2 => A2B
    myEndpoints.back().first.reduce();
  }

  ::std::vector< RT> vec(myHullDims, 0);

  // Always start the hull with the first points at (0,...) and (1,...)
  vec[0] = 1;
  myEndpoints[1].second = PointD(vec.size(), vec.begin(), vec.end());

#ifdef DEBUG_CONVEX_HULL_GENERATOR
  ::std::cout << "Convex hull building simplex\n";
  ::std::cout << "0: " << myEndpoints[0].second << ::std::endl;
  ::std::cout << "1: " << myEndpoints[1].second << ::std::endl;
#endif

  PointD pSum = myEndpoints[1].second;
  vec.assign(vec.size(), RT_ZERO);
  for(int i = 2; i < myHullDims; ++i)
  {
    // Put the new point at the centre of the previous points
    // and 'raise' the last coordinate into the new dimension
    // such as to make the new point unit distance from all the other points
    vec[i - 1] = 1.0;
    for(int j = 0; j < i - 1; ++j)
    {
      vec[j] = pSum[j] / i;
      vec[i - 1] -= vec[j] * vec[j];
    }
    vec[i - 1] = CGAL::sqrt(vec[i - 1]);
    myEndpoints[i].second = PointD(vec.size(), vec.begin(), vec.end());

#ifdef DEBUG_CONVEX_HULL_GENERATOR
    ::std::cout << i << ": " << CGAL::to_double(myEndpoints[i].second[0]) << " " << CGAL::to_double(myEndpoints[i].second[1]) << ::std::endl;
#endif

    pSum += myEndpoints[i].second - CGAL::ORIGIN;
  }
#ifdef DEBUG_CONVEX_HULL_GENERATOR
  ::std::cout << "Convex hull finished building simplex\n";
#endif
}

::boost::optional<ConvexHull::HullTraits::RT>
ConvexHull::distanceToHull(const PointD & p) const
{
  if(!getHull())
    return ::boost::optional<ConvexHull::HullTraits::RT>();

  SSLIB_ASSERT_MSG(p.dimension() == myHullDims, "Point must have same dimension as hull to calculate distance.");

#ifdef DEBUG_CONVEX_HULL_GENERATOR
  ::std::cout << "Calculating distance to point: ";
  printPoint(p);
#endif

  if(isEndpoint(p)) // By definition all endpoints are at 0 on the hull
    return FT_ZERO;

  ::std::vector<HullTraits::FT> direction(myHullDims, FT_ZERO);
  direction[myHullDims - 1] = 1.0;
  HullTraits::Line_d line(p, HullTraits::Direction_d(myHullDims, direction.begin(), direction.end()));

  Hull::Hyperplane_d hyperplane;
  for(Hull::Facet_const_iterator it = const_cast<const Hull *>(myHull.get())->facets_begin(),
      end = const_cast<const Hull *>(myHull.get())->facets_end(); it != end; ++it)
  {
    if(isTopFacet(it))
      continue;

    hyperplane = myHull->hyperplane_supporting(it);
    ::CGAL::Object result = ::CGAL::intersection(hyperplane, line);

    // TODO: Change kernel to use our point type
    const CGAL::Cartesian_d<RT>::Point_d * const intersectionPoint
      = CGAL::object_cast< CGAL::Cartesian_d<RT>::Point_d >(&result);
    if(intersectionPoint &&
        myHull->bounded_side(PointD(myHullDims, intersectionPoint->cartesian_begin(), intersectionPoint->cartesian_end())) == ::CGAL::ON_BOUNDARY)
    {
      return p[myHullDims - 1] - (*intersectionPoint)[myHullDims - 1];
    }
  }

  return ::boost::optional<ConvexHull::HullTraits::RT>(); // Point was not in the space of the hull
}

bool ConvexHull::isTopFacet(Hull::Facet_const_iterator facet) const
{
  for(int i = 0; i < myHull->current_dimension(); ++i)
  {
    if(!isEndpoint(myHull->point_of_facet(facet, i)))
      return false;
  }
  return true;
}

bool ConvexHull::isEndpoint(const PointD & p) const
{
  BOOST_FOREACH(const Endpoint & ep, myEndpoints)
  {
    if(p == ep.second)
      return true;
  }
  return false;
}

void ConvexHull::printPoint(const PointD & p) const
{
  for(int i = 0; i < p.dimension(); ++i)
    ::std::cout << CGAL::to_double(p[i]) << " ";
  ::std::cout << ::std::endl;
}

}
}

#endif // SSLIB_USE_CGAL
