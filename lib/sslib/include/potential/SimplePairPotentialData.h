/*
 * SimplePairPotentialData.h
 *
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef SIMPLE_PAIR_POTENTIAL_DATA_H
#define SIMPLE_PAIR_POTENTIAL_DATA_H

// INCLUDES /////////////////////////////////////////////

#include "common/AtomSpeciesId.h"
#include "common/Structure.h"
#include "potential/StandardData.h"

// FORWARD DECLARATIONS ////////////////////////////////////


// DEFINITION //////////////////////////////////////////////

namespace sstbx
{
namespace potential
{

template <typename FloatType = double>
struct SimplePairPotentialData : public StandardData<FloatType>
{
  /**
  /* A list from 0 to N of the species that each row (and column) of the parameter
  /* matrices corresponds to.  The entries should be unique.
  /*                      Na Cl
  /* epsilon_{ij} = Na (  1  0.5 )
  /*                CL ( 0.5  1  )
  /* speciesMap(0 => Na, 1 => Cl)
  /**/
  typedef ::std::vector< typename ::sstbx::common::AtomSpeciesId::Value>  SpeciesList;

  /**
  /* Any atoms that are not being considered by the potential will be labelled with this.
  /**/
  static const int IGNORE_ATOM = -1;

	SimplePairPotentialData(
    const sstbx::common::Structure & structure,
    const SpeciesList &              speciesList);

	std::vector<int>				species;

};

// IMPLEMENTATION //////////////////////////////////////////

template <typename FloatType>
SimplePairPotentialData<FloatType>::SimplePairPotentialData(
  const sstbx::common::Structure & structure,
  const typename SimplePairPotentialData<FloatType>::SpeciesList & speciesList):
StandardData<FloatType>(structure)
{
  using sstbx::common::AtomSpeciesId;

	// Get the atom species
  std::vector<AtomSpeciesId::Value> strSpecies;
	structure.getAtomSpeciesDescendent(strSpecies);

	// Now populate our species vector
  const size_t numAtoms = strSpecies.size();
	species.resize(numAtoms);

  bool found;
  const size_t numSpecies = speciesList.size();
  AtomSpeciesId::Value currentSpecies;
  for(size_t i = 0; i < numAtoms; ++i)
  {
    currentSpecies = strSpecies[i];
    found = false;
    for(size_t j = 0; j < numSpecies; ++j)
    {
      if(currentSpecies == speciesList[j])
      {
        found = true;
        species[i] = j;
        break;
      }
    }
    if(!found)
    {
      species[i] = IGNORE_ATOM;
    }
  }

}

}
}

#endif /* SIMPLE_PAIR_POTENTIAL_DATA_H */

