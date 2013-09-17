/*
 * SpaceGroup.cpp
 *
 *  Created on: Nov 10, 2011
 *      Author: Martin Uhrin
 */

#include "spl/analysis/SpaceGroup.h"

#include <boost/algorithm/string.hpp>
#include <boost/foreach.hpp>
#include <boost/scoped_array.hpp>

extern "C"
{
#  include <spglib/spglib.h>
}

#include "spl/common/Structure.h"

namespace spl {
namespace analysis {
namespace space_group {

//static const double DEFAULT_PRECISION = 0.05;

bool getSpacegroupInfo(
  SpacegroupInfo & outInfo,
  const common::Structure & structure,
  const double precision)
{
  if(!structure.getUnitCell() || structure.getNumAtoms() == 0)
    return false;

  double lattice[3][3];
  const common::UnitCell * const cell = structure.getUnitCell();
  const::arma::mat33 & orthoMtx = cell->getOrthoMtx();
  for(size_t i = 0; i < 3; ++i)
  {
    for(size_t j = 0; j < 3; ++j)
    {
      // Row-major = column-major
      lattice[i][j] = orthoMtx(i, j);
    }
  }

  const size_t numAtoms = structure.getNumAtoms();
  double (*positions)[3] = new double[numAtoms][3];
  ::arma::mat posMtx;
  structure.getAtomPositions(posMtx);
  cell->cartsToFracInplace(posMtx);
  cell->wrapVecsFracInplace(posMtx);
  for(size_t i = 0; i < numAtoms; ++i)
  {
    for(size_t j = 0; j < 3; ++j)
    {
      // Row-major = column-major
      positions[i][j] = posMtx(j, i);
    }
  }

  ::std::vector<common::AtomSpeciesId::Value> speciesVec;
  structure.getAtomSpecies(speciesVec);
  ::std::map<common::AtomSpeciesId::Value, int> speciesIndices;
  int idx = 0;
  BOOST_FOREACH(const common::AtomSpeciesId::Value & speciesId, speciesVec)
  {
    if(speciesIndices.insert(::std::make_pair(speciesId, idx)).second == true)
      ++idx;
  }

  ::boost::scoped_array<int> species(new int[speciesVec.size()]);
  for(size_t i = 0; i < speciesVec.size(); ++i)
  {
    species[i] = speciesIndices[speciesVec[i]];
  }
  
  // Get the space group
  SpglibDataset * spgData =
    spg_get_dataset(lattice, positions, species.get(), numAtoms, precision);

  // Extract the spacegroup info
  const bool foundSpacegroup = spgData->spacegroup_number != 0;
  if(foundSpacegroup)
  {
    outInfo.number = static_cast<unsigned int>(spgData->spacegroup_number);
    outInfo.iucSymbol = spgData->international_symbol;
    ::boost::algorithm::trim(outInfo.iucSymbol);
    outInfo.hallSymbol = spgData->hall_symbol;
    ::boost::algorithm::trim(outInfo.hallSymbol);
  }

  // Clean up
  spg_free_dataset(spgData);
  spgData = NULL;
  delete [] positions;
  positions = NULL;

  return foundSpacegroup;
}

}
}
}
