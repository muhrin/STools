/*
 * MakeConvexHull.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "blocks/MakeConvexHull.h"

#ifdef SP_USE_QHULL

#include <algorithm>

#include <boost/foreach.hpp>

#include <pipelib/IPipeline.h>

#include <libqhullcpp/Coordinates.h>
#include <libqhullcpp/PointCoordinates.h>
#include <libqhullcpp/QHull.h>
#include <libqhullcpp/UsingLibQhull.h>

#include <common/Structure.h>

#include "common/StructureData.h"

// NAMESPACES ////////////////////////////////


namespace spipe
{
namespace blocks
{

namespace ssc = ::sstbx::common;

MakeConvexHull::MakeConvexHull():
Block<::spipe::StructureDataTyp, ::spipe::SharedDataTyp>("Make convex hull")
{}

void MakeConvexHull::pipelineFinishing()
{
  //myHullPoints.
}

void MakeConvexHull::in(StructureDataTyp & strData)
{
  myStoichiometryData.addStoichiometry(strData);
}

size_t MakeConvexHull::release()
{
  return 0;
}

bool MakeConvexHull::hasData() const
{
  return false;
}


void MakeConvexHull::StoichData::addStoichiometry(
  const StructureDataTyp & strData)
{
  const ssc::Structure * const structure = strData.getStructure();

  if(structure)
  {
    const double energyPerAtom = *strData.enthalpy / structure->getNumAtomsDescendent();

    ::std::vector<ssc::AtomSpeciesId::Value> species;
    structure->getAtomSpeciesDescendent(species);

    size_t maxSpeciesIndex = 0;
    bool found;
    BOOST_FOREACH(const ssc::AtomSpeciesId::Value & strSpecies, species)
    {
      found = false;
      for(size_t i = 0; i < mySpecies.size(); ++i)
      {
        if(strSpecies == mySpecies[i])
        {
          found = true;
          maxSpeciesIndex = ::std::max(maxSpeciesIndex, i);
          break;
        }
      }
      if(!found)
      {
        // Add it to the list
        maxSpeciesIndex = mySpecies.size();
        mySpecies.push_back(strSpecies);
      }
    }

    ::std::vector<size_t> speciesCounts(maxSpeciesIndex + 1);

    for(size_t i = 0; i <= maxSpeciesIndex; ++i)
    {
      speciesCounts[i] = structure->getNumAtomsOfSpecies(mySpecies[i]);
    }

    // Now construct a species matrix for this structure
    HullPoints::iterator hullPoint = myHullPoints.insert(
      myHullPoints.end(),
      HullPoint(
        energyPerAtom,
        SpeciesMatrix(maxSpeciesIndex + 1, maxSpeciesIndex + 1)
      )
    );

    SpeciesMatrix & speciesMtx = hullPoint->speciesMtx;

    for(size_t row = 0; row <= maxSpeciesIndex - 1; ++row)
    {
      for(size_t col = row + 1; col <= maxSpeciesIndex; ++col)
      {
        speciesMtx(row, col) =
          speciesCounts[row] / (speciesCounts[row] + speciesCounts[col]);
      }
    }
  }
}

void MakeConvexHull::StoichData::makeHull() const
{
  // Dimensions = num species + 1 (for energy)
  const size_t dims = mySpecies.size() + 1;
  orgQhull::PointCoordinates points(dims);

  ::std::vector<double> coords(dims);
  BOOST_FOREACH(const HullPoint & hullPoint, myHullPoints)
  {
    points.append(orgQhull::Coordinates(hullPoint.pointCoordinates(dims)));
  }

}

void MakeConvexHull::StoichData::clear()
{
  mySpecies.clear();
  myHullPoints.clear();
}

}
}

#endif /* SP_USE_QHULL */
