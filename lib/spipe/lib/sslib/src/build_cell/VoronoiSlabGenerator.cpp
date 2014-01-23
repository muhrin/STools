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

#include "spl/build_cell/PointSeparator.h"
#include "spl/common/AtomSpeciesDatabase.h" // TODO: Remove
#include "spl/common/ClusterDistanceCalculator.h" // TODO: Remove
#include "spl/common/Structure.h"
#include "spl/math/Matrix.h"

namespace spl {
namespace build_cell {

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
  BOOST_FOREACH(const VoronoiSlabGenerator::SlabRegion & region, myRegions)
    region.generateSites(&dg);

  refineTriangulation(&dg);
  BOOST_FOREACH(const Delaunay::Vertex & vtx,
      boost::make_iterator_range(dg.vertices_begin(), dg.vertices_end()))
  {
    std::cout << vtx.point() << "\n";
  }
  Voronoi vd(dg);

  std::set< Voronoi::Vertex_handle> allVertices(vd.vertices_begin(),
      vd.vertices_end());
  std::vector< common::Atom> atoms;
  BOOST_FOREACH(const VoronoiSlabGenerator::SlabRegion & region, myRegions)
  {
    std::set< Voronoi::Vertex_handle> regionVertices;
    for(std::set< Voronoi::Vertex_handle>::iterator it = allVertices.begin(),
        end = allVertices.end(); it != end; /*increment in body*/)
    {
      std::set< Voronoi::Vertex_handle>::iterator next = it;
      ++next;
      if((*it)->is_valid())
      {
        if(region.withinBoundary((*it)->point()))
        {
          regionVertices.insert(*it);
          allVertices.erase(it);
        }
      }
      // Move on the iterator
      it = next;
    }
    region.generateAtoms(vd, regionVertices, &atoms);
  }

  BOOST_FOREACH(const common::Atom & atom, atoms)
    structure->newAtom(atom).setPosition(
        math::transformCopy(atom.getPosition(), myTransform));

  return true;
}

void
VoronoiSlabGenerator::Slab::addRegion(UniquePtr< SlabRegion>::Type & region)
{
  myRegions.push_back(region);
}

void
VoronoiSlabGenerator::Slab::refineTriangulation(Delaunay * const dg) const
{
  typedef std::vector< std::pair< Delaunay::Point, SiteInfo> > NewVertices;
  std::set< Delaunay::Vertex_handle> vertices;
  for(Delaunay::Vertex_iterator it = dg->vertices_begin(), end =
      dg->vertices_end(); it != end; ++it)
    vertices.insert(it);

  bool changedDg;
  do
  {
    changedDg = false;
    NewVertices toInsert;
    for(std::set< Delaunay::Vertex_handle>::iterator it = vertices.begin();
        it != vertices.end(); /*increment in loop*/)
    {
      std::set< Delaunay::Vertex_handle>::iterator next = it;
      ++next;

      bool isBoundary = false;
      const Delaunay::Vertex_circulator start = dg->incident_vertices(*it);
      Delaunay::Vertex_circulator cl = start;
      do
      {
        if(dg->is_infinite(cl))
          isBoundary = true;
      }
      while(!isBoundary && cl != start);

      if(!isBoundary && !(*it)->info().fixed)
      {
        if((*it)->degree() < 5)
        {
          vertices.erase(*it);
          dg->remove(*it);
          changedDg = true;
        }
        else if((*it)->degree() > 7)
        {
          Delaunay::Point pt(CGAL::to_double((*it)->point().x()),
              CGAL::to_double((*it)->point().y() + 0.1));
          toInsert.push_back(std::make_pair(pt, (*it)->info()));
        }
      }
      it = next;
    }

    BOOST_FOREACH(NewVertices::const_reference pt, toInsert)
    {
      Delaunay::Vertex_handle vtx = dg->insert(pt.first);
      vtx->info() = pt.second;
      vertices.insert(vtx);
    }

    spreadAngles(dg);
    separatePoints(dg);
    if(!toInsert.empty())
    {

//      BOOST_FOREACH(const VoronoiSlabGenerator::SlabRegion & region, myRegions)
//      {
//        std::vector< Delaunay::Vertex_handle> toRemove;
//        for(Delaunay::Vertex_iterator it = dg->vertices_begin(), end =
//            dg->vertices_end(); it != end; ++it)
//        {
//          if(it->info().generatedBy == &region
//              && !region.withinBoundary(it->point()))
//            toRemove.push_back(it);
//        }
//        BOOST_FOREACH(Delaunay::Vertex_handle & vtx, toRemove)
//        {
//          vertices.erase(vtx);
//          dg->remove(vtx);
//        }
//      }

      changedDg = true;
    }
    BOOST_FOREACH(const Delaunay::Vertex & vtx,
        boost::make_iterator_range(dg->vertices_begin(), dg->vertices_end()))
    {
      std::cout << vtx.point() << "\n";
    }
    std::cout << "\n\n";
  }
  while(changedDg);
}

bool
VoronoiSlabGenerator::Slab::separatePoints(Delaunay * const dg) const
{
  // TEMPORARY, CONVERT TO 3D
  static common::Structure LAME_TEMP_STRUCTURE;
  static const common::ClusterDistanceCalculator DIST_CALC(LAME_TEMP_STRUCTURE);
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
        double sep = std::max(sepData.separations(idx, i),
            *x.info().minsep);
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
    dg->move(it,
        Delaunay::Point(sepData.points(0, idx), sepData.points(1, idx)));
    ++idx;
  }

  return result;
}

void
VoronoiSlabGenerator::Slab::spreadAngles(Delaunay * const dg) const
{
  typedef std::map< Delaunay::Vertex_handle, arma::vec2> Forces;

  static const double FORCE_FACTOR = 0.1;

  Forces forces;
  std::pair< Forces::iterator, bool> forcesRet;
  double maxForceSq;

  do
  {
    forces.clear();

    for(Delaunay::Vertex_iterator it = dg->vertices_begin(), end =
        dg->vertices_end(); it != end; ++it)
    {
      const K::Point_2 x0 = it->point();
      arma::vec2 r1, r2, r1Perp, r2Perp, df;
      double forceConstant, angle;

      const Delaunay::Vertex_circulator start = it->incident_vertices();
      Delaunay::Vertex_circulator cl1 = start, cl2 = start;
      if(start.is_empty())
        continue;
      ++cl2;
      do
      {
        if(!(cl1->info().fixed && cl2->info().fixed)
            && !(dg->is_infinite(cl1) || dg->is_infinite(cl2)))
        {
          r1 = utility::toArma(cl1->point() - x0);
          r1 = r1 / std::sqrt(arma::dot(r1, r1));

          r2 = utility::toArma(cl2->point() - x0);
          r2 = r2 / std::sqrt(arma::dot(r2, r2));

          angle = std::abs(std::acos(arma::dot(r1, r2)));

          //std::cout << r1 << r2 << angle << "\n";

          // Counterclockwise
          r1Perp(0) = -r1(1);
          r1Perp(1) = r1(0);
          // Clockwise
          r2Perp(0) = r2(1);
          r2Perp(1) = -r2(0);

          forceConstant =
              cl1->info().fixed || cl2->info().fixed ?
                  FORCE_FACTOR : 0.5 * FORCE_FACTOR;

          if(!cl1->info().fixed)
          {
            df = forceConstant * angle * r1Perp;
            forcesRet = forces.insert(
                std::pair< Delaunay::Vertex_handle, arma::vec2>(cl1, df));
            if(!forcesRet.second)
              forcesRet.first->second += df;
          }
          if(!cl2->info().fixed)
          {
            df = forceConstant * angle * r2Perp;
            forcesRet = forces.insert(
                std::pair< Delaunay::Vertex_handle, arma::vec2>(cl2, df));
            if(!forcesRet.second)
              forcesRet.first->second += df;
          }
        }
        ++cl1;
        ++cl2;
      }
      while(cl1 != start);
    }
    // Now apply all the forces
    maxForceSq = 0.0;
    BOOST_FOREACH(Forces::const_reference f, forces)
    {
      maxForceSq = std::max(maxForceSq, arma::dot(f.second, f.second));
      dg->move(f.first, f.first->point() + utility::toCgalVec< K>(f.second));
    }
  }
  while(maxForceSq > 0.001);
}

VoronoiSlabGenerator::SlabRegion *
new_clone(const VoronoiSlabGenerator::SlabRegion & region)
{
  return region.clone().release();
}

VoronoiSlabGenerator::SlabRegion::SlabRegion(const SlabRegion & toCopy) :
    myBoundary(toCopy.myBoundary), myBasis(toCopy.myBasis->clone())
{

}

void
VoronoiSlabGenerator::SlabRegion::generateAtoms(const Voronoi & vd,
    std::set< Voronoi::Vertex_handle> vertices,
    std::vector< common::Atom> * const atoms) const
{
  myBasis->generateAtoms(vd, vertices, atoms);
}

} // namespace build_cell
} // namespace spl

#endif // SPL_WITH_CGAL
