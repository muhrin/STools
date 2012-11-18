/*
 * MakeConvexHull.h
 *
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef MAKE_CONVEX_HULL_H
#define MAKE_CONVEX_HULL_H

// INCLUDES /////////////////////////////////////////////
#include "StructurePipe.h"

#ifdef SP_USE_QHULL

#include <map>
#include <vector>

#include <armadillo>

#include <pipelib/AbstractSimpleBarrier.h>

#include <common/AtomSpeciesId.h>

// FORWARD DECLARATIONS ////////////////////////////////////

namespace spipe
{
namespace blocks
{

class MakeConvexHull : ::pipelib::AbstractSimpleBarrier<StructureDataTyp, SharedDataTyp>
{
public:

  MakeConvexHull();

  // From Block ///////////
  virtual void in(StructureDataTyp & strData);
  virtual void pipelineFinishing();
  // End from Block ///////

  // From Barrier /////////////////
	virtual size_t release();
	virtual bool hasData() const;
  // End from Barrier //////////////

private:

  class StoichData
  {
  public:

    void addStoichiometry(const StructureDataTyp & strData);

    void clear();

    void makeHull() const;


  private:

    typedef ::arma::mat                     SpeciesMatrix;

    struct HullPoint
    {
      HullPoint(const double _energyPerAtom, const SpeciesMatrix & _speciesMtx):
        energyPerAtom(_energyPerAtom),
        speciesMtx(_speciesMtx)
      {}

      HullPoint & operator =(const HullPoint & rhs)
      {
        energyPerAtom = rhs.energyPerAtom;
        speciesMtx = rhs.speciesMtx;
        return *this;
      }

      ::std::vector<double> pointCoordinates(const size_t numSpecies) const
      {
        const size_t numPoints = numSpecies * (numSpecies - 1) / 2.0 + 1;
        ::std::vector<double> coords(numPoints);

        // Fill the thing with 0
        coords.assign(numPoints + 1, 0.0);
        // Energy is always the first entry
        coords[0] = energyPerAtom;

        // Unroll the matrix
        size_t idx = 1;
        for(size_t row = 0; row < speciesMtx.n_rows - row; ++row)
        {
          for(size_t col = row + 1; col < speciesMtx.n_cols; ++col)
          {
            coords[idx] = speciesMtx(row, col);
            ++idx;
          }
        }
        return coords;
      }

      double              energyPerAtom;
      SpeciesMatrix       speciesMtx;
    };

    typedef ::std::vector< ::sstbx::common::AtomSpeciesId::Value> SpeciesContainer;
    typedef ::std::vector<HullPoint>    HullPoints;

    /** All the species being used for the convex hull */
    SpeciesContainer          mySpecies;
    HullPoints                myHullPoints;

  };

  StoichData  myStoichiometryData;

  

};


}
}

#endif /* SP_USE_QHULL */

#endif /* MAKE_CONVEX_HULL_H */
