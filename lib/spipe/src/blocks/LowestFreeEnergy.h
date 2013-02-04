/*
 * LowestFreeEnergy.h
 *
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef LOWEST_FREE_ENERGY_H
#define LOWEST_FREE_ENERGY_H

// INCLUDES /////////////////////////////////////////////
#include "StructurePipe.h"

#include <map>

#include <boost/noncopyable.hpp>

#include <pipelib/pipelib.h>

#include "SpTypes.h"

// FORWARD DECLARATIONS ////////////////////////////////////


namespace spipe {
namespace blocks {

class LowestFreeEnergy : public SpBarrier, ::boost::noncopyable
{
public:

	LowestFreeEnergy(const size_t keepTopN = 1);
  LowestFreeEnergy(const double keepTopEnergyPercentage);

  // From Block /////////////////
	virtual void in(spipe::common::StructureData & data);
  // End from Block /////////////

  // From Barrier /////////////////
	virtual size_t release();
	virtual bool hasData() const;
  // End from Barrier //////////////

private:
  typedef ::spipe::common::StructureData StructureData;
  typedef ::std::map<double, StructureData *> Structures;

  void keep(StructureData & structure, const double energy);
  void newLowest(StructureData & structure, const double energy);
  double getEnergyCutoff() const;

  const bool myKeepTopNMode;
  const size_t myKeepTopN;
  const double myKeepTopEnergyPercentage;

  Structures myStructures;
};

}
}

#endif /* LOWEST_FREE_ENERGY_H */
