/*
 * ParamPotentialGo.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "blocks/ParamPotentialGo.h"

#include <cstring>
#include <locale>

#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/tokenizer.hpp>

#include <pipelib/IPipeline.h>

// From SSTbx
#include <common/Structure.h>
#include <potential/StandardData.h>
#include <potential/IGeomOptimiser.h>
#include <potential/IParameterisable.h>

#include "common/PipeFunctions.h"
#include "common/StructureData.h"
#include "common/SharedData.h"
#include "common/UtilityFunctions.h"

// NAMESPACES ////////////////////////////////


namespace spipe { namespace blocks {

ParamPotentialGo::ParamPotentialGo(
	::sstbx::potential::IParameterisable & paramPotential,
	::sstbx::potential::IGeomOptimiser & optimiser):
pipelib::Block<StructureDataTyp, SharedDataTyp>("Parameterised potential geometry optimisation"),
myParamPotential(paramPotential),
myOptimiser(optimiser)
{
	using namespace ::boost::algorithm;
	using ::boost::filesystem::path;
	using ::std::fstream;
	using ::std::string;

	// Build up the filename
	string dbName = myParamPotential.getName();
	// TODO: Fix this to do !is_alnum properly
	replace_all(dbName, " "/*!is_alnum()*/, "_");
	to_lower(dbName);
	dbName += ".dat";

	// Open or create the file
	const path dbFilePath(dbName);

	// Open the file for io
	if(exists(dbFilePath))
	{
		myDbStream.open(dbFilePath, fstream::in | fstream::out);
	}
	else
	{
		myDbStream.open(dbFilePath, fstream::out);
	}

	// TODO: READ IN AND POPULATE DATABASE
	//string line;
	//bool found = false;
	//size_t pos = string::npos;;
	//size_t strLen = paramString.length();
	//size_t lineLen = 0;
	//while(!found && dbFile.good())
	//{
	//	::std::getline(dbFile,line);
	//	lineLen = line.length();

	//	// Can we find this parameter string?
	//	pos = line.find(paramString);
	//	// Do some more checks
	//	if(pos + strLen == lineLen || isspace(line.at(pos + strLen)))
	//	{
	//		found = true;
	//	}

	//	if(found)
	//	{
	//		typedef ::boost::tokenizer<boost::char_separator<char> >
	//			Tok;
	//		const ::boost::char_separator<char> sep(" \t");
	//		Tok tok(line, sep);
	//		// Get the first token
	//		myStructureGroup = *tok.begin();
	//	}
	//}
}

ParamPotentialGo::~ParamPotentialGo()
{
	// Close the file - RAII style
	if(myDbStream.is_open())
		myDbStream.close();
}

void ParamPotentialGo::pipelineStarting()
{
  namespace spc = ::spipe::common;

  // The pipeline is starting so try and get the potential parameters
  spc::ObjectData<arma::vec>
    params = spc::getObject(spc::GlobalKeys::POTENTIAL_PARAMS, *myPipeline);

  if(params.first != spc::DataLocation::NONE)
  {
    const arma::vec actualParams = setPotentialParams(*params.second);

    // The potential may have changed the params so reset them in the shared data
    spc::setObject(spc::GlobalKeys::POTENTIAL_PARAMS, params.first, actualParams, *myPipeline);
  }
}

void ParamPotentialGo::in(spipe::common::StructureData & data)
{
  ::boost::shared_ptr<sstbx::potential::StandardData<> > optData;
	if(myOptimiser.optimise(*data.getStructure(), optData))
  {
	  // Copy over information from the optimisation results
	  data.enthalpy.reset(optData->totalEnthalpy);
	  data.stressMtx.reset(optData->stressMtx);

	  myOutput->in(data);
  }
  else
  {
    // The structure failed to geometry optimise properly so drop it
    myPipeline->dropData(data);
  }
}

arma::vec ParamPotentialGo::setPotentialParams(const ::arma::vec & params)
{
	using ::std::string;

	myParamPotential.setParams(params);

  arma::vec actualParams = myParamPotential.getParams();

	// Update the structure group name
	myStructureGroup = ::spipe::common::generateUniqueName();
	myDbStream << myStructureGroup << " " << myParamPotential.getParamString() << std::endl;

  return actualParams;
}


}}

