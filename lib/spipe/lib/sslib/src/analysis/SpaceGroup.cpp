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

extern "C" {
#  include <spglib/spglib.h>
}

#include "spl/common/Structure.h"

namespace spl {
namespace analysis {
namespace space_group {

//static const double DEFAULT_PRECISION = 0.05;

class SpgData
{
public:
  SpgData(SPGCONST double lattice[3][3], SPGCONST double positions[][3],
      const int species[], const size_t numAtoms, const double precision):
        myDataset(spg_get_dataset(lattice, positions, species, static_cast<int>(numAtoms), precision))
  {
  }

  ~SpgData()
  {
    // Clean up
    if(myDataset)
      spg_free_dataset(myDataset);
  }

  const SpglibDataset * const operator ->() const
  {
    return myDataset;
  }
private:
  SpglibDataset * const myDataset;
};

bool
getSpacegroupInfo(SpacegroupInfo & outInfo, const common::Structure & structure,
    const double precision)
{
  if(!structure.getUnitCell() || structure.getNumAtoms() == 0)
    return false;

  double lattice[3][3];
  const common::UnitCell * const cell = structure.getUnitCell();
  const ::arma::mat33 & orthoMtx = cell->getOrthoMtx();
  for(size_t i = 0; i < 3; ++i)
  {
    for(size_t j = 0; j < 3; ++j)
    {
      // Row-major = column-major
      // [row][col] = mtx(row, col)
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
      // [atomIdx][X/Y/Z] = mtx(X/Y/Z, atomIdx)
      positions[i][j] = posMtx(j, i);
    }
  }

  ::std::vector< common::AtomSpeciesId::Value> speciesVec;
  structure.getAtomSpecies(speciesVec);
  ::std::map< common::AtomSpeciesId::Value, int> speciesIndices;
  int idx = 0;
  BOOST_FOREACH(const common::AtomSpeciesId::Value & speciesId, speciesVec)
  {
    if(speciesIndices.insert(::std::make_pair(speciesId, idx)).second == true)
      ++idx;
  }

  ::boost::scoped_array< int> species(new int[speciesVec.size()]);
  for(size_t i = 0; i < speciesVec.size(); ++i)
    species[i] = speciesIndices[speciesVec[i]];

  // Get the space group
  SpgData spgData(lattice, positions, species.get(), numAtoms, precision);

  // Extract the spacegroup info
  const bool foundSpacegroup = spgData->spacegroup_number != 0;
  if(foundSpacegroup)
  {
    outInfo.number = static_cast< unsigned int>(spgData->spacegroup_number);
    outInfo.iucSymbol = spgData->international_symbol;
    ::boost::algorithm::trim(outInfo.iucSymbol);
    outInfo.hallSymbol = spgData->hall_symbol;
    ::boost::algorithm::trim(outInfo.hallSymbol);
  }

  // Clean up
  delete[] positions;
  positions = NULL;

  return foundSpacegroup;
}

}
}
}
