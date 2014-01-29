/*
 * VoronoiSlabGenerator.cpp
 *
 *  Created on: Jan 17, 2014
 *      Author: Martin Uhrin
 */

// INCLUEDES /////////////
#include "spl/build_cell/VoronoiSlabGenerator.h"

#ifdef SPL_WITH_CGAL

#include <boost/range/iterator_range.hpp>

#include <CGAL/centroid.h>

#include "spl/build_cell/PointSeparator.h"
#include "spl/common/ClusterDistanceCalculator.h" // TODO: Remove
#include "spl/common/Structure.h"
#include "spl/math/Matrix.h"

namespace spl {
namespace build_cell {

const int VoronoiSlabGenerator::MAX_ITERATIONS = 10000;

GenerationOutcome
VoronoiSlabGenerator::generateStructure(common::StructurePtr & structure,
    const common::AtomSpeciesDatabase & speciesDb)
{
  structure.reset(new common::Structure());
  BOOST_FOREACH(const Slab & slab, mySlabs)
  {
    slab.generateSlab(structure.get());
  }
  return GenerationOutcome::success();
}

GenerationOutcome
VoronoiSlabGenerator::generateStructure(common::StructurePtr & structure,
    const common::AtomSpeciesDatabase & speciesDb,
    const GenerationSettings & info)
{
  return generateStructure(structure, speciesDb);
}

void
VoronoiSlabGenerator::addSlab(const Slab & slab)
{
  mySlabs.push_back(slab);
}

VoronoiSlabGenerator::Slab::Slab() :
    myTransform(arma::eye(4, 4))
{
}

VoronoiSlabGenerator::Slab::Slab(const arma::mat44 & transform) :
    myTransform(transform)
{
}

bool
VoronoiSlabGenerator::Slab::generateSlab(
    common::Structure * const structure) const
{
  Delaunay dg;
  RegionTickets tickets;
  BOOST_FOREACH(const VoronoiSlabGenerator::SlabRegion & region, myRegions)
  {
    RegionTicket ticket = region.generateSites(&dg);
    if(ticket.valid())
      tickets[&region] = ticket;
  }

  refineTriangulation(tickets, &dg);
  generateAtoms(dg, structure);

  return true;
}

void
VoronoiSlabGenerator::Slab::addRegion(UniquePtr< SlabRegion>::Type & region)
{
  myRegions.push_back(region);
}

bool
VoronoiSlabGenerator::Slab::refineTriangulation(const RegionTickets & tickets,
    Delaunay * const dg) const
{
  typedef std::set< Delaunay::Vertex_handle> RegionSites;
  typedef std::map< const SlabRegion *, RegionSites> UpdateRegions;

  std::vector< Delaunay::Vertex_handle> toRemove;
  UpdateRegions updateRegions;
  bool regionsGood;

  for(int i = 0; i < MAX_ITERATIONS; ++i)
  {
    // Fix up the triangulation
    separatePoints(dg);
    reduceEdges(dg);

    // Find out which region each site belongs to and delete any that have
    // strayed out
    BOOST_FOREACH(const VoronoiSlabGenerator::SlabRegion & region, myRegions)
    {
      const bool haveTicket = tickets.find(&region) != tickets.end();
      toRemove.clear();
      for(Delaunay::Vertex_iterator it = dg->vertices_begin(), end =
          dg->vertices_end(); it != end; ++it)
      {
        if(it->info().generatedBy == &region)
        {
          if(!region.withinBoundary(it->point()))
            toRemove.push_back(it);
          else if(haveTicket)
            updateRegions[&region].insert(it);
        }
      }
      BOOST_FOREACH(Delaunay::Vertex_handle & vtx, toRemove)
        dg->remove(vtx);
    }

    // Check each site that provided a ticket to see if it's happy with the
    // current state of the region
    regionsGood = true;
    BOOST_FOREACH(UpdateRegions::reference update, updateRegions)
    {
      if(!update.first->good(tickets.find(update.first)->second, update.second,
          *dg))
      {
        regionsGood = false;
        break;
      }
    }
    if(regionsGood)
      break;

    // Regions need more refinement
    BOOST_FOREACH(UpdateRegions::reference update, updateRegions)
    {
      update.first->refine(tickets.find(update.first)->second, update.second,
          dg);
      update.second.clear();
    }
  }

  return regionsGood;
}

bool
VoronoiSlabGenerator::Slab::separatePoints(Delaunay * const dg) const
{
  // TEMPORARY, CONVERT TO 3D
  static const common::ClusterDistanceCalculator DIST_CALC;
  SeparationData sepData(dg->number_of_vertices(), DIST_CALC);

  sepData.separations.fill(0.0);
  sepData.points.row(2).fill(0.0); // Set all z values to 0
  size_t idx = 0;
  BOOST_FOREACH(const Delaunay::Vertex & x,
      boost::make_iterator_range(dg->vertices_begin(), dg->vertices_end()))
  {
    sepData.points(0, idx) = CGAL::to_double(x.point().x());
    sepData.points(1, idx) = CGAL::to_double(x.point().y());
    if(x.info().fixed)
      sepData.fixedPoints.insert(idx);

    if(x.info().minsep)
    {
      for(size_t i = 0; i < dg->number_of_vertices(); ++i)
      {
        double sep = std::max(sepData.separations(idx, i), *x.info().minsep);
        sepData.separations(i, idx) = sepData.separations(idx, i) = sep;
      }
    }

    ++idx;
  }

  static const PointSeparator POINT_SEPARATOR(1000, 0.1);
  const bool result = POINT_SEPARATOR.separatePoints(&sepData);

  // Copy back the separated positions, even if it didn't get the positions
  // all the way to the tolerance
  idx = 0;
  for(Delaunay::Vertex_iterator it = dg->vertices_begin(), end =
      dg->vertices_end(); it != end; ++it)
  {
    dg->move_if_no_collision(it,
        Delaunay::Point(sepData.points(0, idx), sepData.points(1, idx)));
    ++idx;
  }

  return result;
}

void
VoronoiSlabGenerator::Slab::reduceEdges(Delaunay * const dg) const
{
  typedef std::map< Delaunay::Vertex_handle, K::Vector_2> Forces;

  static const double FORCE_FACTOR = 0.5;

  Forces forces;
  double maxForceSq;

  do
  {
    forces.clear();

    for(Delaunay::Vertex_iterator it = dg->vertices_begin(), end =
        dg->vertices_end(); it != end; ++it)
    {
      if(it->info().fixed || it->degree() < 3
          || detail::isBoundaryVertex(*dg, it))
        continue;

      const K::Point_2 p0 = it->point();
      K::Vector_2 f(0.0, 0.0), r;

      const Delaunay::Vertex_circulator start = it->incident_vertices();
      Delaunay::Vertex_circulator cl = start;
      if(start.is_empty())
        continue;

      // Build up the list of points that make up the polygon surrounding this vertex
      std::vector< K::Point_2> poly;
      do
      {
        poly.push_back(cl->point());
        ++cl;
      }
      while(cl != start);

      forces[it] = FORCE_FACTOR
          * (CGAL::centroid(poly.begin(), poly.end()) - p0);
    }
    // Now apply all the forces
    maxForceSq = 0.0;
    BOOST_FOREACH(Forces::const_reference f, forces)
    {
      maxForceSq = std::max(maxForceSq,
          CGAL::to_double(f.second.squared_length()));
      dg->move_if_no_collision(f.first, f.first->point() + f.second);
    }
  }
  while(maxForceSq > 0.0001);
}

void
VoronoiSlabGenerator::Slab::generateAtoms(const Delaunay & dg,
    common::Structure * const structure) const
{
  typedef std::set< Delaunay::Face_handle> FacesSet;
  typedef std::list< Delaunay::Face_handle> FacesList;

  FacesList allFaces;
  for(Delaunay::Face_iterator it = dg.faces_begin(), end = dg.faces_end();
      it != end; ++it)
    allFaces.push_back(it);

  std::vector< common::Atom> atoms;
  BOOST_FOREACH(const VoronoiSlabGenerator::SlabRegion & region, myRegions)
  {
    FacesSet regionFaces;
    for(FacesList::iterator it = allFaces.begin(), end = allFaces.end();
        it != end; /*increment in body*/)
    {
      FacesList::iterator next = it;
      ++next;
      Delaunay::Face_handle & face = *it;
      if(face->is_valid())
      {
        if(region.withinBoundary(dg.dual(face)))
        {
          regionFaces.insert(face);
          allFaces.erase(it);
        }
      }
      // Move on the iterator
      it = next;
    }
    region.generateAtoms(dg, regionFaces, &atoms);
  }

  BOOST_FOREACH(const common::Atom & atom, atoms)
    structure->newAtom(atom).setPosition(
        math::transformCopy(atom.getPosition(), myTransform));
}

VoronoiSlabGenerator::SlabRegion::SlabRegion(
    const std::vector< arma::vec2> & boundary, UniquePtr< Basis>::Type & basis) :
    myBasis(basis)
{
  BOOST_FOREACH(const arma::vec2 & r, boundary)
    myBoundary.push_back(utility::toCgalPoint< K>(r));
  SSLIB_ASSERT(!myBoundary.is_empty());
  SSLIB_ASSERT(myBoundary.is_simple());
}

VoronoiSlabGenerator::SlabRegion::SlabRegion(const SlabRegion & toCopy) :
    myBoundary(toCopy.myBoundary), myBasis(toCopy.myBasis->clone())
{
}

const VoronoiSlabGenerator::SlabRegion::Boundary &
VoronoiSlabGenerator::SlabRegion::getBoundary() const
{
  return myBoundary;
}

bool
VoronoiSlabGenerator::SlabRegion::withinBoundary(const arma::vec2 & r) const
{
  return withinBoundary(utility::toCgalPoint< K>(r));
}
bool
VoronoiSlabGenerator::SlabRegion::withinBoundary(
    const Delaunay::Point & r) const
{
  return getBoundary().bounded_side(r) != CGAL::ON_UNBOUNDED_SIDE;
}

void
VoronoiSlabGenerator::SlabRegion::generateAtoms(const Delaunay & dg,
    const std::set< Delaunay::Face_handle> & faces,
    std::vector< common::Atom> * const atoms) const
{
  myBasis->generateAtoms(dg, faces, atoms);
}

VoronoiSlabGenerator::SlabRegion *
new_clone(const VoronoiSlabGenerator::SlabRegion & region)
{
  return region.clone().release();
}

} // namespace build_cell
} // namespace spl

#endif // SPL_WITH_CGAL
