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

#include <armadillo>

#include <pipelib/pipelib.h>

#include <potential/IGeomOptimiser.h>
#include <potential/OptimisationSettings.h>
#include <potential/PotentialData.h>

#include "PipeLibTypes.h"
#include "utility/DataTable.h"
#include "utility/DataTableSupport.h"

// FORWARD DECLARATIONS ////////////////////////////////////

namespace sstbx {
namespace build_cell {
class StructureDescription;
}
namespace common {
class AtomSpeciesDatabase;
}
namespace potential {
class IPotential;
}
}


namespace spipe {
namespace blocks {

class PotentialGo : public pipelib::PipeBlock<StructureDataTyp, SharedDataTyp, SharedDataTyp>
{
public:

	PotentialGo(
    const ::sstbx::potential::IGeomOptimiser &  optimiser,
    const bool                                  writeOutput = true
  );

	PotentialGo(
    const ::sstbx::potential::IGeomOptimiser &  optimiser,
    const ::sstbx::potential::OptimisationSettings & optimisationParams,
    const bool                                  writeOutput = true
  );

  // From Block ///////////////////////////////
  virtual void pipelineInitialising();
  // End from Block //////////////////////////

  // From PipeBlock ///////////////////////////
	virtual void in(spipe::common::StructureData & data);
  // End from PipeBlock ///////////////////////

protected:

  virtual void copyOptimisationResults(
    const sstbx::potential::PotentialData & optData,
    sstbx::common::Structure & structure);

  void updateTable(const sstbx::common::Structure & structure);

  // Should we write information about structures being optimised
  // to file.
  const bool                               myWriteOutput;

	const sstbx::potential::IGeomOptimiser & myOptimiser;
  const ::sstbx::potential::OptimisationSettings  myOptimisationParams;
  ::arma::mat33                            myExternalPressure;

  // Use a table to store data about structure that are being optimised
  ::spipe::utility::DataTableSupport       myTableSupport;

};

}
}

#endif /* POTENTIAL_GO_H */
