/*
 * ConvexHullGenerator.h
 *
 *  Created on: Jul 9, 2013
 *      Author: Martin Uhrin
 */

#ifndef CONVEX_HULL_GENERATAOR_H
#define CONVEX_HULL_GENERATAOR_H

// INCLUDES ////////////
#include "SSLib.h"

#ifdef SSLIB_USE_CGAL

#include <map>
#include <vector>
#include <string>

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
#include "common/Structure.h"
#include "utility/HeterogeneousMapKey.h"

// DEFINITION ///////////////////////

namespace sstbx {

// FORWARD DECLARATIONS ///////

namespace analysis {

class HullEntry;

class ConvexHullGenerator : public AbsConvexHullGenerator, ::boost::noncopyable
{
public:
//#ifdef CGAL_USE_GMP
//  typedef CGAL::Quotient<CGAL::Gmpfr> RT;
//#else
  typedef CGAL::Quotient<CORE::Expr> RT;
//#endif
  typedef CGAL::Cartesian_d<RT> HullTraits;

public:

  typedef CGAL::Convex_hull_d<HullTraits> ConvexHull;
  typedef HullTraits::Point_d PointD;
  typedef HullTraits::Vector_d VectorD;
  typedef ::std::vector< ::std::string> Endpoints;

  class HullEntry
  {
  public:
    HullEntry(const common::Structure::Composition & composition, const HullTraits::FT value);

    const common::Structure::Composition & getComposition() const;
    const HullTraits::FT getValue() const;
    bool isEndpoint() const;
  private:
    common::Structure::Composition myComposition;
    HullTraits::FT myValue;
    bool myIsEndpoint;
  };

  template <typename InputIterator>
  static UniquePtr<ConvexHullGenerator>::Type makeHull(InputIterator first, InputIterator last);

  ConvexHullGenerator(const Endpoints & endpoints);
  ConvexHullGenerator(const Endpoints & endpoints, utility::Key<double> & convexProperty);
  virtual ~ConvexHullGenerator() {}

  bool addStructure(const common::Structure & structure);
  template <typename InputIterator>
  void addStructures(InputIterator first, InputIterator last);

  const ConvexHull * getHull();

private:

  typedef ::std::map< ::std::string, HullTraits::FT> ChemicalPotentials;
  typedef ::std::vector<HullEntry> HullEntries;

  HullEntries::iterator generateEntry(const common::Structure & structure);
  void updateChemicalPotential(const ::std::string & endpointSpecies, const HullTraits::FT value);
  PointD generateHullPoint(const HullEntry & entry);
  void generateHull();
  bool canGenerate() const;
  void initSimplexCoordinates();

  const Endpoints myEndpoints;
  const utility::Key<double> myConvexProperty;
  ChemicalPotentials myChemicalPotentials;
  const int myHullDims;
  HullEntries myEntries;
  ::boost::scoped_ptr<ConvexHull> myHull;
  ::std::vector<VectorD> mySimplexCoordinates;
};

template <typename InputIterator>
void ConvexHullGenerator::addStructures(InputIterator first, InputIterator last)
{
  for(InputIterator it = first; it != last; ++it)
    addStructure(*it);
}

template <typename InputIterator>
UniquePtr<ConvexHullGenerator>::Type ConvexHullGenerator::makeHull(InputIterator first, InputIterator last)
{
  UniquePtr<ConvexHullGenerator>::Type generator;

  ::std::set< ::std::string> speciesSet;
  ::std::vector< ::std::string> speciesVec;
  for(InputIterator it = first; it != last; ++it)
  {
    it->getAtomSpecies(speciesVec);
    speciesSet.insert(speciesVec.begin(), speciesVec.end());
    speciesVec.clear();
  }
  speciesVec.insert(speciesVec.begin(), speciesSet.begin(), speciesSet.end());

  generator.reset(new ConvexHullGenerator(speciesVec));
  generator->addStructures(first, last);

  return generator;
}

}
}

#endif // SSLIB_USE_CGAL

#endif /* CONVEX_HULL_GENERATAOR_H */
