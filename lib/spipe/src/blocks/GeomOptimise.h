/*
 * GeomOptimise.h
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef GEOM_OPTIMISE_H
#define GEOM_OPTIMISE_H

// INCLUDES /////////////////////////////////////////////
#include "StructurePipe.h"

#include <boost/noncopyable.hpp>

#include <armadillo>

#include <pipelib/pipelib.h>

#include <spl/potential/IGeomOptimiser.h>
#include <spl/potential/OptimisationSettings.h>
#include <spl/potential/PotentialData.h>
#include <spl/potential/Types.h>

#include "SpTypes.h"
#include "utility/DataTable.h"
#include "utility/DataTableSupport.h"

// FORWARD DECLARATIONS ////////////////////////////////////

namespace spl {
namespace common {
class AtomSpeciesDatabase;
}
namespace potential {
class IPotential;
}
}

namespace spipe {
namespace blocks {

class GeomOptimise : public PipeBlock, ::boost::noncopyable
{
public:
  GeomOptimise(::spl::potential::IGeomOptimiserPtr optimiser);

  GeomOptimise(::spl::potential::IGeomOptimiserPtr optimiser,
      const ::spl::potential::OptimisationSettings & optimisationParams);

  // From Block ///////////////////////////////
  virtual void
  pipelineInitialising();
  // End from Block //////////////////////////

  // From PipeBlock ///////////////////////////
  virtual void
  in(common::StructureData * const data);
  // End from PipeBlock ///////////////////////

  bool
  getWriteOutput() const;
  void
  setWriteOutput(const bool write);

protected:
  ::spl::potential::IGeomOptimiser &
  getOptimiser();
  ::spipe::utility::DataTableSupport &
  getTableSupport();

  void
  updateTable(const spl::common::Structure & structure);

  // Should we write information about structures being optimised
  // to file.
  bool myWriteOutput;

  const ::spl::potential::OptimisationSettings myOptimisationParams;

private:
  const spl::potential::IGeomOptimiserPtr myOptimiser;

  // Use a table to store data about structure that are being optimised
  ::spipe::utility::DataTableSupport myTableSupport;
};

}
}

#endif /* GEOM_OPTIMISE_H */
