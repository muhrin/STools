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

namespace sstbx
{
namespace analysis
{

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

ConvexHull::VectorD
ConvexHull::composition(const VectorD & vec) const
{
  return VectorD(myHullDims - 1, vec.cartesian_begin(), vec.cartesian_end() - 1);
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
  const HullEntries::iterator it = myEntries.insert(myEntries.end(),
      HullEntry(structure.getComposition(), *value, id, isEndpoint));
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
  // Need chemical potentials for all endpoints
  SSLIB_ASSERT(canGenerate());

  const common::AtomsFormula & composition = entry.getComposition();

  int totalAtoms = 0;
  HullTraits::FT totalMuNAtoms = 0.0;
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
    }
  }
  tempVec.assign(v.cartesian_begin(), v.cartesian_end());
  // The last hull coordinate is always the 'convex property', usually the energy
  if(!entry.isEndpoint())
    tempVec[dims() - 1] = (entry.getValue() - totalMuNAtoms) / totalAtoms;

  PointD point(myHullDims, tempVec.begin(), tempVec.end());
  point.setId(entry.getId());

#ifdef DEBUG_CONVEX_HULL_GENERATOR
  ::std::cout << "Convex value: " << entry.getValue() << " mu_alpha N_alpha: " << totalMuNAtoms << " total atoms: " << totalAtoms << ::std::endl;
  ::std::cout << "Hull entry: " << point << ::std::endl;
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
  vec.assign(vec.size(), 0.0);
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

}
}

#endif // SSLIB_USE_CGAL
