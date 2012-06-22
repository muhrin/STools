/*
 * RandomCellGenerator.h
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef RANDOM_CELL_GENERATOR_H
#define RANDOM_CELL_GENERATOR_H

// INCLUDES //////////////////////////////////////////////
#include "build_cell/ICellGenerator.h"

// DEFINES ///////////////////////////////////////////////


// FORWARD DECLARES //////////////////////////////////////
namespace sstbx { namespace build_cell {
	class CellDescription;
	template <typename FloatType>
	class RandomCellDescription;
}}

namespace sstbx { namespace build_cell {

template <typename FloatType = double>
class RandomCellGenerator : public ICellGenerator<FloatType>
{
public:

	static const FloatType DEFAULT_VOLUME;
	static const FloatType DEFAULT_VOL_DELTA;
	static const FloatType DEFAULT_MAX_LENGTH_RATIO;
	static const FloatType DEFAULT_MIN_ANGLE;
	static const FloatType DEFAULT_MAX_ANGLE;
	static const FloatType MIN_RAND_LATTICE_LENGTH;

	virtual ::sstbx::common::AbstractFmidCell<FloatType> *
    generateCell(const RandomCellDescription<FloatType> & cellDesc) const;

private:

	enum LatticeParams {A, B, C, ALPHA, BETA, GAMMA, TOTAL_PARAMS};

	FloatType generateAngle(const RandomCellDescription<FloatType> & cellDesc) const;

	FloatType generateLength(
    const RandomCellDescription<FloatType> & cellDesc,
    const FloatType a,
    const FloatType b = 0.0) const;

	FloatType generateVolume(
    const RandomCellDescription<FloatType> & cellDesc,
    const FloatType currentVolume) const;

};

}}

// IMPLEMENTATION //////////////////////////////////////

// INCLUDES //////////////////////////////////////////
#include<iostream>

#include "RandomCellDescription.h"
#include "common/AbstractFmidCell.h"
#include "common/UnitCellFactory.h"
#include "common/Utils.h"

#define UNSET 0.0

namespace sstbx { namespace build_cell {

// Constants ///////////////////////////////////////////////////////////
template <typename FloatType>
const FloatType RandomCellGenerator<FloatType>::DEFAULT_VOLUME		= 1.0;
template <typename FloatType>
const FloatType RandomCellGenerator<FloatType>::DEFAULT_VOL_DELTA	= 0.2;
template <typename FloatType>
const FloatType RandomCellGenerator<FloatType>::DEFAULT_MAX_LENGTH_RATIO = 2.0;
template <typename FloatType>
const FloatType RandomCellGenerator<FloatType>::DEFAULT_MIN_ANGLE = 45;
template <typename FloatType>
const FloatType RandomCellGenerator<FloatType>::DEFAULT_MAX_ANGLE = 135;
template <typename FloatType>
const FloatType RandomCellGenerator<FloatType>::MIN_RAND_LATTICE_LENGTH = 1e-4;


template <typename FloatType>
::sstbx::common::AbstractFmidCell<FloatType> *
RandomCellGenerator<FloatType>::generateCell(const RandomCellDescription<FloatType> & cellDesc) const
{
	::sstbx::common::AbstractFmidCell<FloatType> * cell = NULL;

	// New set of lattice parameters
	double latticeParams[6];

	// Set the lattice lengths
	latticeParams[A] = cellDesc.myA ? *cellDesc.myA : 0.5;
	latticeParams[B] = cellDesc.myB ? *cellDesc.myB : generateLength(cellDesc, latticeParams[A]);
	latticeParams[C] = cellDesc.myC ? *cellDesc.myC : generateLength(cellDesc, latticeParams[A], latticeParams[B]);

	// Set the lattice angles
	latticeParams[ALPHA]	= cellDesc.myAlpha ? *cellDesc.myAlpha : generateAngle(cellDesc);
	latticeParams[BETA]		= cellDesc.myBeta ? *cellDesc.myBeta : generateAngle(cellDesc);
	latticeParams[GAMMA]	= cellDesc.myGamma ? *cellDesc.myGamma : generateAngle(cellDesc);

	// Make sure the length ratio constraint is adhered to
	if(cellDesc.myMaxLengthRatio)
	{
    const FloatType maxLengthRatio = *cellDesc.myMaxLengthRatio;

		// Find the shortest lattice vector
		LatticeParams min = latticeParams[A] < latticeParams[B] ? A : B;
		min = latticeParams[min] < latticeParams[C] ? min : C;

		for(size_t i = A; i <= C; ++i)
		{
			if(i != min && (latticeParams[i]/latticeParams[min]) > maxLengthRatio)
			{
				// Cap the length at the maximum
				latticeParams[i] = latticeParams[min] * maxLengthRatio;
			}
		}
	}

	// Make sure the min/max angle constraint is adhered to
	if(cellDesc.myMinAngle)
	{
    const FloatType minAngle = *cellDesc.myMinAngle;
		for(size_t i = ALPHA; i <= GAMMA; ++i)
		{
			latticeParams[i] = latticeParams[i] < minAngle ? minAngle : latticeParams[i];
		}
	}
	if(cellDesc.myMaxAngle)
	{
    const FloatType maxAngle = *cellDesc.myMaxAngle;
		for(size_t i = ALPHA; i <= GAMMA; ++i)
		{
			latticeParams[i] = latticeParams[i] > maxAngle ? maxAngle : latticeParams[i];
		}
	}

	// Create the unit cell
	cell = new sstbx::common::AbstractFmidCell<FloatType>(latticeParams);

	// Finally set the volume
	cell->setVolume(generateVolume(cellDesc, cell->getVolume()));

	return cell;
}


template <typename FloatType>
FloatType
RandomCellGenerator<FloatType>::generateAngle(
  const RandomCellDescription<FloatType> & cellDesc) const
{
  const FloatType min = cellDesc.myMinAngle ? *cellDesc.myMinAngle : DEFAULT_MIN_ANGLE;
  const FloatType max = cellDesc.myMaxAngle ? *cellDesc.myMaxAngle : DEFAULT_MAX_ANGLE;
	return sstbx::common::randDouble(min, max);
}

template <typename FloatType>
FloatType RandomCellGenerator<FloatType>::generateLength(
  const RandomCellDescription<FloatType> & cellDesc,
  const FloatType a,
  const FloatType b) const
{
  const FloatType maxLengthRatio = cellDesc.myMaxLengthRatio ? *cellDesc.myMaxLengthRatio : DEFAULT_MAX_LENGTH_RATIO;
	const FloatType shortest = ::std::max(a, b) / maxLengthRatio;
	const FloatType longest = b == 0 ? maxLengthRatio * a : ::std::min(a, b) * maxLengthRatio;

	return sstbx::common::randDouble(shortest, longest);
}

template <typename FloatType>
FloatType RandomCellGenerator<FloatType>::generateVolume(
  const RandomCellDescription<FloatType> & cellDesc,
  const FloatType currentVolume) const
{
	// Find the target volume, the volume constraint superceeds a volume
	// inferred by the cell definition
	const FloatType targetVol = cellDesc.myVolume ? *cellDesc.myVolume : currentVolume;
	const FloatType volDelta = cellDesc.myVolDelta ? *cellDesc.myVolDelta : 0.0;

	return sstbx::common::randDouble(targetVol * (1.0 - volDelta), targetVol * (1.0 + volDelta));
}

}}

#endif /* RANDOM_CELL_GENERATOR_H */
