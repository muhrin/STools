/*
 * Histogram.cpp
 *
 *  Created on: Nov 10, 2011
 *      Author: Martin Uhrin
 */

#include "analysis/ConvexHullGenerator.h"

#ifdef SSLIB_USE_CGAL

#include <boost/foreach.hpp>

#include "common/StructureProperties.h"

//#define DEBUG_CONVEX_HULL_GENERATOR

namespace sstbx {
namespace analysis {

ConvexHullGenerator::ConvexHullGenerator(const Endpoints & endpoints):
    myEndpoints(endpoints), myConvexProperty(common::structure_properties::general::ENTHALPY),
    myHullDims(endpoints.size()), mySimplexCoordinates(endpoints.size(), VectorD(endpoints.size() - 1))
{
  SSLIB_ASSERT_MSG(myEndpoints.size() >= 2, "Need at least two endpoints to make convex hull.");
  initSimplexCoordinates();
}

ConvexHullGenerator::ConvexHullGenerator(const Endpoints & endpoints, utility::Key<double> & convexProperty):
    myEndpoints(endpoints), myConvexProperty(convexProperty), myHullDims(endpoints.size()),
    mySimplexCoordinates(endpoints.size(), VectorD(endpoints.size() - 1))
{
  SSLIB_ASSERT_MSG(myEndpoints.size() >= 2, "Need at least two endpoints to make convex hull.");
  initSimplexCoordinates();
}

bool ConvexHullGenerator::addStructure(const common::Structure & structure)
{
  return generateEntry(structure) != myEntries.end();
}

const ConvexHullGenerator::ConvexHull * ConvexHullGenerator::getHull()
{
  if(!myHull.get() && canGenerate())
    generateHull();

  return myHull.get();
}

ConvexHullGenerator::HullEntry::HullEntry(const common::Structure::Composition & composition, const HullTraits::FT value):
    myComposition(composition), myValue(value)
{
  myIsEndpoint = true;
  bool foundNonZero = false;
  BOOST_FOREACH(common::Structure::Composition::const_reference x, myComposition)
  {
    if(!foundNonZero && x.second != 0)
      foundNonZero = true;
    else if(foundNonZero && x.second != 0)
    {
      myIsEndpoint = false;
      break;
    }
  }
}

const common::Structure::Composition & ConvexHullGenerator::HullEntry::getComposition() const
{
  return myComposition;
}

const ConvexHullGenerator::HullTraits::FT ConvexHullGenerator::HullEntry::getValue() const
{
  return myValue;
}

bool ConvexHullGenerator::HullEntry::isEndpoint() const
{
  return myIsEndpoint;
}

ConvexHullGenerator::HullEntries::iterator
ConvexHullGenerator::generateEntry(const common::Structure & structure)
{
  if(structure.getNumAtoms() == 0)
    return myEntries.end();

  // Check if the structure has a value for the property that will form the 'depth' of the hull
  const double * const value = structure.getProperty(myConvexProperty);
  if(!value)
    return myEntries.end();

  const HullEntries::iterator it = myEntries.insert(myEntries.end(), HullEntry(structure.getComposition(), *value));
  if(it->isEndpoint())
  {
    ::std::string nonZeroElement;
    BOOST_FOREACH(common::Structure::Composition::const_reference x, it->getComposition())
    {
      if(x.second != 0)
      {
        nonZeroElement = x.first;
        break;
      }
    }
    updateChemicalPotential(nonZeroElement, *value / static_cast<double>(structure.getNumAtoms()));
  }

  return it;
}

void ConvexHullGenerator::updateChemicalPotential(const ::std::string & endpointSpecies, const HullTraits::FT value)
{
  ChemicalPotentials::iterator it = myChemicalPotentials.find(endpointSpecies);
  if(it == myChemicalPotentials.end())
    myChemicalPotentials[endpointSpecies] = value;
  else
  {
    if(it->second > value)
    {
      it->second = value;
      myHull.reset();   // Hull needs to be re-generated
    }
  }
}

ConvexHullGenerator::PointD ConvexHullGenerator::generateHullPoint(const HullEntry & entry)
{
  // Need chemical potentials for all endpoints
  SSLIB_ASSERT(canGenerate());

  ::std::vector<HullTraits::FT> hullCoords(myHullDims);

  const common::Structure::Composition & composition = entry.getComposition();

  int totalAtoms = 0;
  HullTraits::FT totalMuNAtoms = 0.0;
  // TODO: Rewrite this loop in terms of our endpoints instead of composition as the
  // composition may contain species that we're not interested in
  BOOST_FOREACH(common::Structure::Composition::const_reference x, composition)
  {
    totalAtoms += x.second;
    totalMuNAtoms += myChemicalPotentials[x.first] * x.second;
  }

  // Create a vector that is the weighted sum of the vectors of composition
  // simplex
  VectorD v(myHullDims - 1);
  common::Structure::Composition::const_iterator it;
  for(int i = 0; i < myEndpoints.size(); ++i)
  {
    it = composition.find(myEndpoints[i]);
    if(it != composition.end())
    {
      VectorD scaled = mySimplexCoordinates[i];
      scaled *= HullTraits::FT(it->second, totalAtoms);
#ifdef DEBUG_CONVEX_HULL_GENERATOR
      ::std::cout << "Adding endpoint: " << scaled << " weight: " << HullTraits::FT(it->second, totalAtoms) << ::std::endl;
#endif
      v += scaled;
    }
  }
  for(int i = 0; i < myHullDims - 1; ++i)
    hullCoords[i + 1] = v[i];


  // The first hull coordinate is always the 'convex property', usually the energy
  hullCoords[0] = (entry.getValue() - totalMuNAtoms) / totalAtoms;

  PointD point(myHullDims, hullCoords.begin(), hullCoords.end());

#ifdef DEBUG_CONVEX_HULL_GENERATOR
  ::std::cout << "Hull entry: " << point << ::std::endl;
#endif

  return point;
}

void ConvexHullGenerator::generateHull()
{
  SSLIB_ASSERT(canGenerate());

  myHull.reset(new ConvexHull(myHullDims));
  BOOST_FOREACH(HullEntries::const_reference entry, myEntries)
  {
    myHull->insert(generateHullPoint(entry));
  }
}

bool ConvexHullGenerator::canGenerate() const
{
  return myChemicalPotentials.size() == myEndpoints.size();
}

void ConvexHullGenerator::initSimplexCoordinates()
{
  ::std::vector<RT> vec(myEndpoints.size() - 1, 0);

  // Always start the hull with the first points at (0,0) and (1,0)
  vec[0] = 1.0;
  mySimplexCoordinates[1] = VectorD(vec.size(), vec.begin(), vec.end());

#ifdef DEBUG_CONVEX_HULL_GENERATOR
  ::std::cout << "Convex hull building simplex\n";
  ::std::cout << "0: " << mySimplexCoordinates[0] << ::std::endl;
  ::std::cout << "1: " << mySimplexCoordinates[1] << ::std::endl;
#endif

  VectorD pSum = mySimplexCoordinates[1];
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
    mySimplexCoordinates[i] = VectorD(vec.size(), vec.begin(), vec.end());

#ifdef DEBUG_CONVEX_HULL_GENERATOR
  ::std::cout << i << ": " << CGAL::to_double(mySimplexCoordinates[i][0]) << " " << CGAL::to_double(mySimplexCoordinates[i][1]) << ::std::endl;
#endif

    pSum += mySimplexCoordinates[i];
  }
#ifdef DEBUG_CONVEX_HULL_GENERATOR
  ::std::cout << "Convex hull finished building simplex\n";
#endif
}

}
}

#endif // SSLIB_USE_CGAL
