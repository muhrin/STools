/*
 * ConvexHull.cpp
 *
 *  Created on: Nov 10, 2011
 *      Author: Martin Uhrin
 */

#include "analysis/ConvexHull.h"

#ifdef SSLIB_USE_CGAL

#include <boost/foreach.hpp>

#include "common/StructureProperties.h"

//#define DEBUG_CONVEX_HULL_GENERATOR

namespace sstbx {
namespace analysis {

ConvexHull::ConvexHull(const EndpointLabels & labels):
  myConvexProperty(common::structure_properties::general::ENTHALPY),
  myHullDims(labels.size())
{
  SSLIB_ASSERT_MSG(labels.size() >= 2, "Need at least two endpoints to make convex hull.");
  initEndpoints(labels);
}

ConvexHull::ConvexHull(const EndpointLabels & labels, utility::Key<double> & convexProperty):
  myConvexProperty(convexProperty), myHullDims(labels.size())
{
  SSLIB_ASSERT_MSG(labels.size() >= 2, "Need at least two endpoints to make convex hull.");
  initEndpoints(labels);
}

ConvexHull::PointId ConvexHull::addStructure(const common::Structure & structure)
{
  return generateEntry(structure);
}

int ConvexHull::dims() const
{
  return myHullDims;
}

const ConvexHull::Hull * ConvexHull::getHull() const
{
  if(!myHull.get() && canGenerate())
    generateHull();

  return myHull.get();
}

ConvexHull::VectorD ConvexHull::composition(const VectorD & vec) const
{
  return VectorD(myHullDims - 1, vec.cartesian_begin(), vec.cartesian_end() - 1);
}

ConvexHull::PointD ConvexHull::composition(const PointD & point) const
{
  return PointD(myHullDims - 1, point.cartesian_begin(), point.cartesian_end() - 1);
}

ConvexHull::EndpointsConstIterator ConvexHull::endpointsBegin() const
{
  return myEndpoints.begin();
}

ConvexHull::EndpointsConstIterator ConvexHull::endpointsEnd() const
{
  return myEndpoints.end();
}

::boost::optional<bool> ConvexHull::isStable(const PointD & point) const
{
  if(!myHull.get())
    return ::boost::optional<bool>();

  return point[dims() - 1] <= 0 && myHull->bounded_side(point) == CGAL::ON_BOUNDARY;
}

ConvexHull::HullEntry::HullEntry(const common::AtomsFormula & composition,
    const HullTraits::FT value, const ConvexHull::PointId id):
    myComposition(composition), myValue(value), myId(id)
{}

const common::AtomsFormula & ConvexHull::HullEntry::getComposition() const
{
  return myComposition;
}

const ConvexHull::HullTraits::FT ConvexHull::HullEntry::getValue() const
{
  return myValue;
}

const ConvexHull::PointId & ConvexHull::HullEntry::getId() const
{
  return myId;
}

ConvexHull::PointId
ConvexHull::generateEntry(const common::Structure & structure)
{
  // Check if the structure has a value for the property that will form the 'depth' of the hull
  const double * const value = structure.getProperty(myConvexProperty);
  if(!value)
    return -1;

  // Check that the structure contains at least hull endpoint
  const common::AtomsFormula & composition = structure.getComposition();
  bool containsEndpoint = false, isEndpoint = true;
  int endpointFormulaUnits;
  common::AtomsFormula endpoint;
  ::std::pair<int, int> endpointUnits;
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
    }
  }
  if(!containsEndpoint)
    return -1; // Doesn't contain any endpoints so it's nowhere in relation to hull

  const PointId id = myEntries.size();
  const HullEntries::iterator it = myEntries.insert(myEntries.end(), HullEntry(composition, *value, id));
  if(isEndpoint)
    updateChemicalPotential(endpoint, HullTraits::FT(*value, static_cast<double>(endpointFormulaUnits)));

  return id;
}

void ConvexHull::updateChemicalPotential(const common::AtomsFormula & endpointFormula, const HullTraits::FT value)
{
  ChemicalPotentials::iterator it = myChemicalPotentials.find(endpointFormula);
  if(it == myChemicalPotentials.end())
    myChemicalPotentials[endpointFormula] = value;
  else
  {
    if(it->second > value)
    {
      it->second = value;
      myHull.reset();   // Hull needs to be re-generated
    }
  }
}

ConvexHull::PointD ConvexHull::generateHullPoint(const HullEntry & entry) const
{
  // Need chemical potentials for all endpoints
  SSLIB_ASSERT(canGenerate());

  const common::AtomsFormula & composition = entry.getComposition();

  int totalAtoms = 0;
  HullTraits::FT totalMuNAtoms = 0.0;
  int numAtoms;
  BOOST_FOREACH(Endpoints::const_reference endpoint, myEndpoints)
  {
    numAtoms = composition.numberOf(endpoint.first).first;
    if(numAtoms != 0)
    {
      totalAtoms += numAtoms;
      totalMuNAtoms += myChemicalPotentials.find(endpoint.first)->second * HullTraits::FT(numAtoms);
    }
  }

  // Create a vector that is the weighted sum of the vectors of composition
  // simplex
  ::std::vector<HullTraits::FT> tempVec(myHullDims);
  VectorD v(myHullDims);
  for(int i = 0; i < myEndpoints.size(); ++i)
  {
    numAtoms = composition.numberOf(myEndpoints[i].first).first;
    if(numAtoms != 0)
    {
      VectorD scaled = myEndpoints[i].second - CGAL::ORIGIN;
      scaled *= HullTraits::FT(numAtoms, totalAtoms);
#ifdef DEBUG_CONVEX_HULL_GENERATOR
      ::std::cout << "Adding weighted hull endpoint: " << scaled << " weight: " << HullTraits::FT(numAtoms, totalAtoms) << ::std::endl;
#endif
      v += scaled;
    }
  }
  tempVec.assign(v.cartesian_begin(), v.cartesian_end());
  // The last hull coordinate is always the 'convex property', usually the energy
  tempVec[dims() - 1] = (entry.getValue() - totalMuNAtoms) / totalAtoms;

  PointD point(myHullDims, tempVec.begin(), tempVec.end());
  point.setId(entry.getId());

#ifdef DEBUG_CONVEX_HULL_GENERATOR
  ::std::cout << "Convex value: " << entry.getValue() << " mu_alpha N_alpha: " << totalMuNAtoms << " total atoms: " << totalAtoms << ::std::endl;
  ::std::cout << "Hull entry: " << point << ::std::endl;
#endif

  return point;
}

void ConvexHull::generateHull() const
{
  SSLIB_ASSERT(canGenerate());

  myHull.reset(new Hull(myHullDims));
  BOOST_FOREACH(HullEntries::const_reference entry, myEntries)
  {
    const PointD & p = generateHullPoint(entry);
    if(p[dims() - 1] <= 0)
      myHull->insert(p);
  }
  if(!myHull->is_valid(false))
    myHull.reset();
}

bool ConvexHull::canGenerate() const
{
  return myChemicalPotentials.size() == myEndpoints.size();
}

void ConvexHull::initEndpoints(const EndpointLabels & labels)
{
  for(int i = 0; i < labels.size(); ++i)
  {
    myEndpoints.push_back(::std::make_pair(labels[i], PointD(myHullDims)));
    // Make sure we're using the reduced, i.e. most simple, formula
    // e.g. A4B2 => A2B
    myEndpoints.back().first.reduce();
  }

  ::std::vector<RT> vec(myHullDims, 0);

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
