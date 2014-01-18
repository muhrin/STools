/*
 * VoronoiSlabRegions.cpp
 *
 *  Created on: Jan 17, 2014
 *      Author: Martin Uhrin
 */

// INCLUEDES /////////////
#include "spl/build_cell/VoronoiSlabRegions.h"

#include "spl/common/Atom.h"

namespace spl {
namespace build_cell {

LatticeRegion::LatticeRegion(const std::vector< arma::vec2> & boundary,
    const arma::vec2 & vecA, const arma::vec2 & vecB,
    const std::vector< std::string> & basis):
        SlabRegion(boundary)
{
  myLattice[0] = vecA;
  myLattice[1] = vecB;
  myBasis = basis;
}

LatticeRegion::LatticeRegion(const LatticeRegion & toCopy) :
    SlabRegion(toCopy), myLattice(toCopy.myLattice), myBasis(toCopy.myBasis)
{
}

void
LatticeRegion::generateSites(std::vector< Site> * const points) const
{
  const CGAL::Bbox_2 & bbox = getBoundary().bbox();
  arma::vec2 r0;
  r0(0) = bbox.xmin();
  r0(1) = bbox.xmax();

  // Precondition: inBBox(r, bbox) == true
  arma::vec2 r;
  for(r = r0; inBBox(r, bbox); r += myLattice[0])
    generateLine(r, myLattice[1], bbox, points);

  for(r = r0 - myLattice[0]; inBBox(r, bbox); r -= myLattice[0])
    generateLine(r, myLattice[1], bbox, points);
}

void
LatticeRegion::generateAtoms(const Voronoi & vd,
    std::set< Voronoi::Vertex_handle> vertices,
    std::vector< common::Atom> * const atoms) const
{
  while(!vertices.empty())
  {
    placeAtoms(0, vertices.begin(), &vertices, atoms);
  }
}

std::auto_ptr< VoronoiSlabGenerator::SlabRegion>
LatticeRegion::clone() const
{
  return std::auto_ptr< VoronoiSlabGenerator::SlabRegion>(
      new LatticeRegion(*this));
}

void
LatticeRegion::placeAtoms(const size_t basisIdx,
    const std::set< Voronoi::Vertex_handle>::iterator & it,
    std::set< Voronoi::Vertex_handle> * const vertices,
    std::vector< common::Atom> * const atoms) const
{
  arma::vec3 r;
  r(0) = CGAL::to_double((*it)->point().x());
  r(1) = CGAL::to_double((*it)->point().y());
  r(2) = 0.0;
  common::Atom atom(myBasis[basisIdx]);
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
        placeAtoms(basisIdx + 1 % myBasis.size(), neighIt, vertices, atoms);
    }
    ++he; // Move on by 2 to get to the next edge
    ++he;
  }
  while(he != start);
}

void
LatticeRegion::generateLine(const arma::vec2 & r0, const arma::vec2 & dr,
    const CGAL::Bbox_2 & box, std::vector< Site> * const points) const
{
  // Precondition: inBBox(r0, box)

  // Scan positive multiples of dr
  arma::vec2 r = r0;
  for(r = r0; inBBox(r, box); r += dr)
  {
    if(withinBoundary(r))
      points->push_back(Site(r, true));
  }

  // Scan negative multiples of dr
  for(r = r0 - dr; inBBox(r, box); r -= dr)
  {
    if(withinBoundary(r))
      points->push_back(Site(r, true));
  }
}

bool
LatticeRegion::inBBox(const arma::vec2 & r, const CGAL::Bbox_2 & bbox) const
{
  return r(0) >= bbox.xmin() && r(0) <= bbox.xmax() && r(1) >= bbox.ymin()
      && r(1) <= bbox.ymax();
}

}
}
