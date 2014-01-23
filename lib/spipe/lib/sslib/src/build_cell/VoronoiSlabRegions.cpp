/*
 * VoronoiSlabRegions.cpp
 *
 *  Created on: Jan 17, 2014
 *      Author: Martin Uhrin
 */

// INCLUEDES /////////////
#include "spl/build_cell/VoronoiSlabRegions.h"

#ifdef SPL_WITH_CGAL

#include "spl/common/Atom.h"
#include "spl/math/Geometry.h"
#include "spl/math/NumberAlgorithms.h"
#include "spl/math/Random.h"
#include "spl/utility/Armadillo.h"

namespace spl {
namespace build_cell {

using arma::endr;

LatticeRegion::LatticeRegion(const ConstructionInfo & info,
    UniquePtr< Basis>::Type & basis) :
    SlabRegion(info.boundary, basis), myStartPoint(info.startPoint)
{
  myLattice[0] = info.vecA;
  myLattice[1] = info.vecB;
}

LatticeRegion::LatticeRegion(const LatticeRegion & toCopy) :
    SlabRegion(toCopy), myLattice(toCopy.myLattice), myStartPoint(
        toCopy.myStartPoint)
{
}

void
LatticeRegion::generateSites(Delaunay * const dg) const
{
  const CGAL::Bbox_2 & bbox = getBoundary().bbox();
  arma::vec2 x0;
  if(myStartPoint)
    x0 = *myStartPoint;
  else
    x0 << bbox.xmin() << endr << bbox.ymin(); // Use the bottom left corner of the AABB

  // Find out the min and max multiples of each lattice vector needed to cover
  // the bounding box entirely.
  const LatticeMultiples latVec0Multiples = getLatticeMultiples(x0,
      myLattice[0], myLattice[1]);
  const LatticeMultiples latVec1Multiples = getLatticeMultiples(x0,
      myLattice[1], myLattice[0]);

  for(int i = latVec0Multiples.lower(); i <= latVec0Multiples.upper(); ++i)
  {
    const arma::vec2 & r = x0 + static_cast< double>(i) * myLattice[0];
    generateLine(r, myLattice[1], latVec1Multiples, bbox, dg);
  }
}

std::auto_ptr< VoronoiSlabGenerator::SlabRegion>
LatticeRegion::clone() const
{
  return std::auto_ptr< VoronoiSlabGenerator::SlabRegion>(
      new LatticeRegion(*this));
}

LatticeRegion::LatticeMultiples
LatticeRegion::getLatticeMultiples(const arma::vec2 & r0,
    const arma::vec2 & latticeVecP, const arma::vec2 & latticeVecQ) const
{
  using arma::dot;
  using std::ceil;
  using std::max;
  using std::min;
  using std::sqrt;

  const CGAL::Bbox_2 & bbox = getBoundary().bbox();
  const math::Line2 pLine(r0, r0 + latticeVecP);

  int x0 = std::numeric_limits< int>::max();
  int x1 = std::numeric_limits< int>::min();
  // Go around all four corners of the bounding box
  arma::vec boxPt;
  { // bottom left
    boxPt << bbox.xmin() << endr << bbox.ymin();

    const double numMultiples = getLatticeVecMultiple(r0, latticeVecP, pLine,
        math::Line2(boxPt, boxPt + latticeVecQ));

    x0 = min(x0, static_cast< int>(numMultiples));
    x1 = max(x1, static_cast< int>(numMultiples));
  }
  { // top left
    boxPt << bbox.xmin() << endr << bbox.ymax();

    const double numMultiples = getLatticeVecMultiple(r0, latticeVecP, pLine,
        math::Line2(boxPt, boxPt + latticeVecQ));

    x0 = min(x0, static_cast< int>(numMultiples));
    x1 = max(x1, static_cast< int>(numMultiples));
  }
  { // top right
    boxPt << bbox.xmax() << endr << bbox.ymax();

    const double numMultiples = getLatticeVecMultiple(r0, latticeVecP, pLine,
        math::Line2(boxPt, boxPt + latticeVecQ));

    x0 = min(x0, static_cast< int>(numMultiples));
    x1 = max(x1, static_cast< int>(numMultiples));
  }
  { // bottom right
    boxPt << bbox.xmax() << endr << bbox.ymin();

    const double numMultiples = getLatticeVecMultiple(r0, latticeVecP, pLine,
        math::Line2(boxPt, boxPt + latticeVecQ));

    x0 = min(x0, static_cast< int>(numMultiples));
    x1 = max(x1, static_cast< int>(numMultiples));
  }

  return LatticeMultiples(x0, x1);
}

int
LatticeRegion::getLatticeVecMultiple(const arma::vec2 & x0,
    const arma::vec2 & latticeVec, const math::Line2 & latticeLine,
    const math::Line2 & intersectionLine) const
{
  using arma::dot;
  using std::ceil;
  using std::sqrt;

  const arma::vec2 & dr = math::intersectionPoint(latticeLine, intersectionLine)
      - x0;

  return math::sgn(dot(dr, latticeVec))
      * ceil(sqrt(dot(dr, dr) / dot(latticeVec, latticeVec)));
}

void
LatticeRegion::generateLine(const arma::vec2 & r0, const arma::vec2 & dr,
    const LatticeMultiples & range, const CGAL::Bbox_2 & box,
    Delaunay * const dg) const
{
  arma::vec2 r;
  for(int i = range.lower(); i <= range.upper(); ++i)
  {
    r = r0 + static_cast< double>(i) * dr;
    if(inBBox(r, box) && withinBoundary(r))
    {
      Delaunay::Vertex_handle vtx = dg->insert(
          utility::toCgalPoint< Delaunay::Geom_traits>(r));
      vtx->info().fixed = true;
      vtx->info().generatedBy = this;
    }
  }
}

bool
LatticeRegion::inBBox(const arma::vec2 & r, const CGAL::Bbox_2 & bbox) const
{
  return r(0) >= bbox.xmin() && r(0) <= bbox.xmax() && r(1) >= bbox.ymin()
      && r(1) <= bbox.ymax();
}

RandomRegion::RandomRegion(const std::vector< arma::vec2> & boundary,
    const int numPoints, const double minsep, UniquePtr< Basis>::Type & basis) :
    SlabRegion(boundary, basis), myNumPoints(numPoints), myMinsep(minsep)
{
}

void
RandomRegion::generateSites(Delaunay * const dg) const
{
  const CGAL::Bbox_2 & bbox = getBoundary().bbox();
  arma::vec2 pt;
  for(int numGenerated = 0; numGenerated < myNumPoints; /*increment in body*/)
  {
    pt << math::randu(bbox.xmin(), bbox.xmax()) << endr
        << math::randu(bbox.ymin(), bbox.ymax());
    if(withinBoundary(pt))
    {
      Delaunay::Vertex_handle vtx = dg->insert(
          utility::toCgalPoint< Delaunay::Geom_traits>(pt));
      vtx->info().fixed = false;
      vtx->info().minsep = 2.4;
      vtx->info().generatedBy = this;
      ++numGenerated;
    }
  }
}

std::auto_ptr< VoronoiSlabGenerator::SlabRegion>
RandomRegion::clone() const
{
  return std::auto_ptr< SlabRegion>(new RandomRegion(*this));
}

OrderedBasis::OrderedBasis(const std::vector< std::string> & species) :
    mySpecies(species)
{
}

bool
OrderedBasis::generateAtoms(const Voronoi & vd,
    std::set< Voronoi::Vertex_handle> vertices,
    std::vector< common::Atom> * const atoms) const
{
  while(!vertices.empty())
    placeAtoms(0, vertices.begin(), &vertices, atoms);
  return true;
}

UniquePtr< VoronoiSlabGenerator::SlabRegion::Basis>::Type
OrderedBasis::clone() const
{
  return UniquePtr< VoronoiSlabGenerator::SlabRegion::Basis>::Type(
      new OrderedBasis(*this));
}

void
OrderedBasis::placeAtoms(const size_t basisIdx,
    const std::set< Voronoi::Vertex_handle>::iterator & it,
    std::set< Voronoi::Vertex_handle> * const vertices,
    std::vector< common::Atom> * const atoms) const
{
  arma::vec3 r;
  r(0) = CGAL::to_double((*it)->point().x());
  r(1) = CGAL::to_double((*it)->point().y());
  r(2) = 0.0;
  common::Atom atom(mySpecies[basisIdx]);
  atom.setPosition(r);
  atoms->push_back(atom);

  const Voronoi::Halfedge_around_vertex_circulator start =
      (*it)->incident_halfedges();
  Voronoi::Halfedge_around_vertex_circulator he = start;
  Voronoi::Vertex_handle neighbour;
  vertices->erase(it); // Erase the iterator from the set now
  do
  {
    if(he->has_source())
    {
      neighbour = he->source();
      // Check if the neighbour is in our set of vertices
      std::set< Voronoi::Vertex_handle>::iterator neighIt = vertices->find(
          neighbour);
      if(neighIt != vertices->end())
        placeAtoms((basisIdx + 1) % mySpecies.size(), neighIt, vertices, atoms);
    }
    ++he; // Move on by 2 to get to the next edge
    ++he;
  }
  while(!vertices->empty() && he != start);
}

}
}

#endif // SPL_WITH_CGAL
