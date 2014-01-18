/*
 * VoronoiSlabGenerator.cpp
 *
 *  Created on: Jan 17, 2014
 *      Author: Martin Uhrin
 */

// INCLUEDES /////////////
#include "spl/build_cell/VoronoiSlabGenerator.h"

#include <boost/range/iterator_range.hpp>

#include "spl/build_cell/PointSeparator.h"
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
    const common::AtomSpeciesDatabase & speciesDb, const GenerationSettings & info)
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
  std::vector< Site> sites;
  BOOST_FOREACH(const VoronoiSlabGenerator::SlabRegion & region, myRegions)
    region.generateSites(&sites);

  // TEMPORARY, CONVERT TO 3D
  static common::Structure LAME_TEMP_STRUCTURE;
  static const common::ClusterDistanceCalculator DIST_CALC(LAME_TEMP_STRUCTURE);
  SeparationData sepData(sites.size(), DIST_CALC);

  sepData.points.row(2).fill(0.0); // Set all z values to 0
  for(size_t i = 0; i < sites.size(); ++i)
  {
    sepData.points(0, i) = sites[i].first(0);
    sepData.points(1, i) = sites[i].first(1);
    if(sites[i].second)
      sepData.fixedPoints.insert(i);
  }
  sepData.separations.fill(1.0);

  static const PointSeparator POINT_SEPARATOR;
  if(!POINT_SEPARATOR.separatePoints(&sepData))
    return false;

  std::vector< Voronoi::Site_2> cgalSites;
  BOOST_FOREACH(const Site & site, sites)
    cgalSites.push_back(Voronoi::Site_2(site.first(0), site.first(1)));
  Voronoi vd(cgalSites.begin(), cgalSites.end());

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
        if(region.getBoundary().bounded_side((*it)->point())
            != CGAL::ON_UNBOUNDED_SIDE)
        {
          regionVertices.insert(*it);
          allVertices.erase(it);
        }
        region.generateAtoms(vd, regionVertices, &atoms);
      }
      // Move on the iterator
      it = next;
    }
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

VoronoiSlabGenerator::SlabRegion *
new_clone(const VoronoiSlabGenerator::SlabRegion & region)
{
  return region.clone().release();
}

}
}
