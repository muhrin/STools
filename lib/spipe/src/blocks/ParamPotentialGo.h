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

#include "pipelib/AbstractSimpleBlock.h"

#include <armadillo>

#include <boost/filesystem/fstream.hpp>
#include <map>

// FORWARD DECLARATIONS ////////////////////////////////////

namespace sstbx
{
namespace potential
{
class IParameterisable;
class IGeomOptimiser;
}
}


namespace spipe { namespace blocks {

class ParamPotentialGo : public pipelib::AbstractSimpleBlock<StructureDataTyp, SharedDataTyp>
{
public:
	ParamPotentialGo(
		::sstbx::potential::IParameterisable & paramPotential,
		::sstbx::potential::IGeomOptimiser & optimiser);

	virtual ~ParamPotentialGo();

	virtual void pipelineStarting();

	virtual void in(spipe::common::StructureData & data);

private:

	typedef ::std::map<std::string, ::std::string> PotDbTyp;

  arma::vec setPotentialParams(const ::arma::vec & params);

	::std::fstream & getDbStream();

	::sstbx::potential::IParameterisable &	myParamPotential;
	const sstbx::potential::IGeomOptimiser & myOptimiser;

	boost::filesystem::fstream				myDbStream;
	PotDbTyp								myDb;

	::std::string							myStructureGroup;
};

}}

#endif /* PARAM_POTENTIAL_GO_H */
