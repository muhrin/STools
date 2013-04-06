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

#include <potential/IGeomOptimiser.h>
#include <potential/OptimisationSettings.h>
#include <potential/PotentialData.h>

#include "SpTypes.h"
#include "potential/Types.h"
#include "utility/DataTable.h"
#include "utility/DataTableSupport.h"

// FORWARD DECLARATIONS ////////////////////////////////////

namespace sstbx {
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

	PotentialGo(
    ::sstbx::potential::IGeomOptimiserPtr optimiser,
    const bool writeOutput = true
  );

	PotentialGo(
    ::sstbx::potential::IGeomOptimiserPtr optimiser,
    const ::sstbx::potential::OptimisationSettings & optimisationParams,
    const bool writeOutput = true
  );

  // From Block ///////////////////////////////
  virtual void pipelineInitialising();
  // End from Block //////////////////////////

  // From PipeBlock ///////////////////////////
	virtual void in(spipe::common::StructureData & data);
  // End from PipeBlock ///////////////////////

protected:
  ::sstbx::potential::IGeomOptimiser & getOptimiser();
  ::spipe::utility::DataTableSupport & getTableSupport();

  void updateTable(const sstbx::common::Structure & structure);

  // Should we write information about structures being optimised
  // to file.
  const bool myWriteOutput;

  const ::sstbx::potential::OptimisationSettings  myOptimisationParams;

private:
  const sstbx::potential::IGeomOptimiserPtr myOptimiser;

  // Use a table to store data about structure that are being optimised
  ::spipe::utility::DataTableSupport myTableSupport;
};

}
}

#endif /* POTENTIAL_GO_H */
