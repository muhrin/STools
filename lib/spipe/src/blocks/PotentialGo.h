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

#include <pipelib/AbstractSimpleBlock.h>

#include <potential/StandardData.h>

#include "utility/DataTable.h"
#include "utility/DataTableSupport.h"

// FORWARD DECLARATIONS ////////////////////////////////////

namespace sstbx
{
namespace potential
{
class IGeomOptimiser;
}
}


namespace spipe { namespace blocks {

class PotentialGo : public pipelib::AbstractSimpleBlock<StructureDataTyp, SharedDataTyp>
{
public:
	PotentialGo(
    const ::sstbx::potential::IGeomOptimiser &  optimiser,
    const ::arma::mat33 * const                 externalPressure = NULL,
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
    const sstbx::potential::StandardData<> & optData,
    spipe::common::StructureData & strData);

  void updateTable(const ::spipe::StructureDataTyp & strData);

  // Should we write information about structures being optimised
  // to file.
  const bool                               myWriteOutput;

	const sstbx::potential::IGeomOptimiser & myOptimiser;
  ::arma::mat33                            myExternalPressure;

  // Use a table to store data about structure that are being optimised
  ::spipe::utility::DataTableSupport       myTableSupport;
};

}}

#endif /* POTENTIAL_GO_H */
