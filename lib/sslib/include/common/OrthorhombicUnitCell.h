/*
 * OrthorhombicUnitCell.h
 *
 * 
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef ORTHORHOMBIC_UNIT_CELL_H
#define ORTHORHOMBIC_UNIT_CELL_H

// INCLUDES /////////////////////////////////////////////
#include "common/AbstractFmidCell.h"

// FORWARD DECLARATIONS ////////////////////////////////////

namespace sstbx { namespace common {

template<typename FloatType = double>
class OrthorhombicUnitCell : public AbstractFmidCell
{
public:
	explicit OrthorhombicUnitCell(
		const FloatType a, const FloatType b, const FloatType c,
		const FloatType alpha, const FloatType beta, const FloatType gamma):
	AbstractFmidCell(a, b, c, alpha, beta, gamma)
	{}

	virtual FloatType getDistanceSqMinimumImg(
		AbstractFmidCell::Vec3 const & a, AbstractFmidCell::Vec3 const & b) const
	{
		AbstractFmidCell::Vec3 r = b - a;
		// Fractionalise and wrap to maximum 0.5 in any direction
		fractionaliseInplace(r);
		r(0) = fmod(r(0), 0.5);
		r(1) = fmod(r(1), 0.5);
		r(2) = fmod(r(2), 0.5);
		// Orthogonalise back
		orthogonaliseInplace(r);

		return arma::dot(r, r);
	}

	virtual AbstractFmidCell::Vec3 getVecMinimumImg(
		AbstractFmidCell::Vec3 const & a, AbstractFmidCell::Vec3 const & b) const
	{
		AbstractFmidCell::Vec3 r = b - a;
		// Fractionalise and wrap to maximum 0.5 in any direction
		fractionaliseInplace(r);
		r(0) = fmod(r(0), 0.5);
		r(1) = fmod(r(1), 0.5);
		r(2) = fmod(r(2), 0.5);
		// Orthogonalise back
		orthogonaliseInplace(r);

		return r;
	}

};

}}

#endif /* ORTHORHOMBIC_UNIT_CELL_H */
