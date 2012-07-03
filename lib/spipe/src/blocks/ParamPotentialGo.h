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

#include <pipelib/AbstractSimpleBlock.h>

#include "blocks/PotentialGo.h"
#include "utility/DataTable.h"
#include "utility/DataTableSupport.h"


// FORWARD DECLARATIONS ////////////////////////////////////

namespace sstbx
{
namespace potential
{
class IParameterisable;
class IGeomOptimiser;
}
}


namespace spipe
{
namespace blocks
{

class ParamPotentialGo : public PotentialGo
{
public:
	ParamPotentialGo(
		::sstbx::potential::IParameterisable & paramPotential,
		const ::sstbx::potential::IGeomOptimiser & optimiser,
    const ::arma::mat33 * const externalPressure = NULL,
    const bool                  writeOutput = true);

  // From Block /////////////////////////
	virtual void pipelineStarting();
  // End from Block ////////////////////

private:

  virtual void copyOptimisationResults(
    const sstbx::potential::StandardData<> & optData,
    spipe::common::StructureData & strData);

  arma::vec setPotentialParams(const ::arma::vec & params);

	::sstbx::potential::IParameterisable &      myParamPotential;
  ::arma::vec                                 myCurrentParams;
};

}}

#endif /* PARAM_POTENTIAL_GO_H */
