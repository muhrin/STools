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
#include <map>
#include <set>

#include <armadillo>

#include "common/AtomSpeciesId.h"
#include "common/Structure.h"
#include "potential/StandardData.h"

// FORWARD DECLARATIONS ////////////////////////////////////


// DEFINITION //////////////////////////////////////////////

namespace sstbx { namespace potential {

template <typename FloatType = double>
struct SimplePairPotentialData : public StandardData<FloatType>
{
	SimplePairPotentialData(const sstbx::common::Structure & structure);

	std::vector<size_t>				species;

};

// IMPLEMENTATION //////////////////////////////////////////

template <typename FloatType>
SimplePairPotentialData<FloatType>::SimplePairPotentialData(const sstbx::common::Structure & structure):
StandardData<FloatType>(structure)
{
  using sstbx::common::AtomSpeciesId;

	// Get the atom species
  std::vector<AtomSpeciesId::Value> strSpecies;
	structure.getAtomSpeciesDescendent(strSpecies);

	// Copy the species over to a set and sort so we know which
	// species we have overall and in ascending order
  std::set<AtomSpeciesId::Value> speciesSet(strSpecies.begin(), strSpecies.end());

  std::vector<AtomSpeciesId::Value> sortedSpecies(speciesSet.begin(), speciesSet.end());
	std::sort(sortedSpecies.begin(), sortedSpecies.end());

	// Now populate our species vector
	species.resize(strSpecies.size());
	for(size_t i = 0; i < sortedSpecies.size(); ++i)
	{
		for(size_t j = 0; j < strSpecies.size(); ++j)
		{
			if(strSpecies[j] == sortedSpecies[i])
			{
				species[j] = i;
			}
		}
	}
}

}}

#endif /* SIMPLE_PAIR_POTENTIAL_DATA_H */
