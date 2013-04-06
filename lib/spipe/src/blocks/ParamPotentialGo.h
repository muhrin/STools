/*
 * ParamPotentialGo.h
 * Parameterisable Potential Geometry Optimisation
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef PARAM_POTENTIAL_GO_H
#define PARAM_POTENTIAL_GO_H

// INCLUDES /////////////////////////////////////////////
#include "StructurePipe.h"

#include <map>

#include <boost/filesystem/fstream.hpp>

#include <armadillo>

#include <pipelib/pipelib.h>

#include "SpTypes.h"
#include "blocks/PotentialGo.h"
#include "utility/DataTable.h"
#include "utility/DataTableSupport.h"


// FORWARD DECLARATIONS ////////////////////////////////////

namespace sstbx {
namespace potential {
class IParameterisable;
class IGeomOptimiser;
}
}


namespace spipe {
namespace blocks {

class ParamPotentialGo : public PotentialGo
{
public:

	ParamPotentialGo(
		::sstbx::potential::IGeomOptimiserPtr optimiser,
    const bool writeOutput = true);

	ParamPotentialGo(
		::sstbx::potential::IGeomOptimiserPtr optimiser,
    const ::sstbx::potential::OptimisationSettings & optimisationParams,
    const bool writeOutput = true);

  // From Block /////////////////////////
	virtual void pipelineStarting();
  // End from Block ////////////////////

  // From PipeBlock ///////////////////////////
	virtual void in(spipe::common::StructureData & data);
  // End from PipeBlock ///////////////////////

private:

  typedef ::std::vector<double> PotentialParams;

  void init();

  void setPotentialParams(const PotentialParams & params);

	::sstbx::potential::IParameterisable * myParamPotential;
  PotentialParams myCurrentParams;
};

}
}

#endif /* PARAM_POTENTIAL_GO_H */
