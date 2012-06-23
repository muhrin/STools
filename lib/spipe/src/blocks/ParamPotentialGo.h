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

class ParamPotentialGo : public pipelib::AbstractSimpleBlock<StructureDataTyp, SharedDataTyp>
{
public:
	ParamPotentialGo(
		::sstbx::potential::IParameterisable & paramPotential,
		::sstbx::potential::IGeomOptimiser & optimiser);

  // From Block /////////////////////////
  virtual void pipelineInitialising();
	virtual void pipelineStarting();
  // End from Block ////////////////////

	virtual void in(spipe::common::StructureData & data);

private:

	typedef ::std::map<std::string, ::std::string> PotDbTyp;

  arma::vec setPotentialParams(const ::arma::vec & params);

  void updateTable(const ::spipe::StructureDataTyp & strData);

	::sstbx::potential::IParameterisable &      myParamPotential;
	const sstbx::potential::IGeomOptimiser &    myOptimiser;
  ::arma::vec                                 myCurrentParams;

  ::spipe::utility::DataTableSupport          myTableSupport;
};

}}

#endif /* PARAM_POTENTIAL_GO_H */
