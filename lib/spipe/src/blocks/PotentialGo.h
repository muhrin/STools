/*
 * PotentialGo.h
 * Potential Geometry Optimisation
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef POTENTIAL_GO_H
#define POTENTIAL_GO_H

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

class PotentialGo : public SpPipeBlock, ::boost::noncopyable
{
public:

  PotentialGo(::spl::potential::IGeomOptimiserPtr optimiser, const bool writeOutput = true);

  PotentialGo(::spl::potential::IGeomOptimiserPtr optimiser,
      const ::spl::potential::OptimisationSettings & optimisationParams, const bool writeOutput =
          true);

  // From Block ///////////////////////////////
  virtual void
  pipelineInitialising();
  // End from Block //////////////////////////

  // From PipeBlock ///////////////////////////
  virtual void
  in(spipe::common::StructureData & data);
  // End from PipeBlock ///////////////////////

protected:
  ::spl::potential::IGeomOptimiser &
  getOptimiser();
  ::spipe::utility::DataTableSupport &
  getTableSupport();

  void
  updateTable(const spl::common::Structure & structure);

  // Should we write information about structures being optimised
  // to file.
  const bool myWriteOutput;

  const ::spl::potential::OptimisationSettings myOptimisationParams;

private:
  const spl::potential::IGeomOptimiserPtr myOptimiser;

  // Use a table to store data about structure that are being optimised
  ::spipe::utility::DataTableSupport myTableSupport;
};

}
}

#endif /* POTENTIAL_GO_H */
