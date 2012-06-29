/*
 * DetermineSpaceGroup.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "blocks/DetermineSpaceGroup.h"

#include <vector>

#include <boost/algorithm/string.hpp>
#include <boost/scoped_array.hpp>

#include <armadillo>

extern "C"
{
#include <spglib/spglib.h>
}

#include <common/AbstractFmidCell.h>
#include <common/AtomSpeciesId.h>
#include <common/Structure.h>

// Local includes
#include "common/StructureData.h"

// NAMESPACES ////////////////////////////////


namespace spipe
{
namespace blocks
{
namespace common = ::spipe::common;
namespace ssc = ::sstbx::common;

DetermineSpaceGroup::DetermineSpaceGroup():
::pipelib::Block< ::spipe::StructureDataTyp, ::spipe::SharedDataTyp>("Determine space group")
{}

void DetermineSpaceGroup::in(StructureDataTyp & data)
{
  // Express structure data in form that spglib can work with
  const ssc::Structure * const structure = data.getStructure();

  if(structure)
  {
    double lattice[3][3];
    const::arma::mat33 & orthoMtx = structure->getUnitCell()->getOrthoMtx();
    for(size_t i = 0; i < 3; ++i)
    {
      for(size_t j = 0; j < 3; ++j)
      {
        // Row-major = column-major
        lattice[i][j] =
          orthoMtx(i, j);
      }
    }

    const size_t numAtoms = structure->getNumAtoms();
    double (*positions)[3] = new double[numAtoms][3];
    ::arma::mat posMtx;
    structure->getAtomPositionsDescendent(posMtx);
    structure->getUnitCell()->fractionaliseInplace(posMtx);
    for(size_t i = 0; i < numAtoms; ++i)
    {
      for(size_t j = 0; j < 3; ++j)
      {
        // Row-major = column-major
        positions[i][j] = posMtx(j, i);
      }
    }

    ::std::vector<ssc::AtomSpeciesId::Value> speciesVec;
    structure->getAtomSpeciesDescendent(speciesVec);
    ::boost::scoped_array<int> species(new int[speciesVec.size()]);
    for(size_t i = 0; i < speciesVec.size(); ++i)
    {
      species[i] = speciesVec[i].ordinal();
    }

    // Get the space group
    SpglibDataset * spgData =
      spg_get_dataset(lattice, positions, species.get(), numAtoms, 0.05);

    data.objectsStore.insert(
      common::StructureObjectKeys::SPACEGROUP_NUMBER,
      (unsigned int)spgData->spacegroup_number);

    ::std::string spacegroupSymbol(spgData->international_symbol);
    ::boost::algorithm::trim(spacegroupSymbol);

    data.objectsStore.insert(
      common::StructureObjectKeys::SPACEGROUP_SYMBOL,
      spacegroupSymbol);

    // Clean up
    spg_free_dataset(spgData);
    delete [] positions;
  }


  myOutput->in(data);
}

}
}
