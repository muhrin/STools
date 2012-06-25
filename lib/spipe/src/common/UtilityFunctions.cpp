/*
 * UtilityFunctions.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: muhrin
 */

// INCLUDES //////////////////////////////////
#include "UtilityFunctions.h"

#include <boost/lexical_cast.hpp>

// From SSTbx
#include <io/AdditionalData.h>

// From local
#include "common/StructureData.h"

// NAMESPACES ////////////////////////////////

namespace spipe { namespace common {

ProcessId getProcessId()
{
	return NS_BOOST_IPC_DETAIL::get_current_process_id();
}

std::string generateUniqueName()
{
	// Use boost as portable way to get the process id
	const ProcessId	processId	= getProcessId();
	const time_t	currTime	= time(NULL);

	// Build up the name
	std::stringstream ss;	//create a stringstream
	ss << processId << "-" << currTime;

	return ss.str();
}

void generateIoDataFromStructure(
  const ::spipe::common::StructureData & strData,
  sstbx::io::AdditionalData & ioData)
{
	if(strData.enthalpy)
	{
		ioData.enthalpy.reset(*strData.enthalpy);
	}
	if(strData.name)
	{
		ioData.name.reset(*strData.name);
	}
	if(strData.pressure)
	{
		ioData.pressure.reset(*strData.pressure);
	}
	if(strData.spaceGroup)
	{
		ioData.spaceGroup.reset(*strData.spaceGroup);
	}
	if(strData.timesFound)
	{
		ioData.timesFound.reset(*strData.timesFound);
	}
  const ::std::string * const spacegroup = strData.objectsStore.find(
    StructureObjectKeys::SPACEGROUP_SYMBOL);
  if(spacegroup)
  {
    ioData.spaceGroup.reset(*spacegroup);
  }
}

void generateStructureDataFromIo(
  const sstbx::io::AdditionalData & ioData,
  spipe::common::StructureData & strData)
{
	if(ioData.enthalpy)
	{
		strData.enthalpy.reset(*ioData.enthalpy);
	}
	if(ioData.name)
	{
		strData.name.reset(*ioData.name);
	}
	if(ioData.pressure)
	{
		strData.pressure.reset(*ioData.pressure);
	}
	if(ioData.spaceGroup)
	{
		strData.spaceGroup.reset(*ioData.spaceGroup);
	}
	if(ioData.timesFound)
	{
		strData.timesFound.reset(*ioData.timesFound);
	}
}

bool parseParamString(
  const std::string & str,
  double &            from,
  double &            step,
  unsigned int &      nSteps
)
{
  using std::string;
  using boost::lexical_cast;

  if(str.empty())
    return false;

  const size_t plusPos = str.find("+");
  const size_t timesPos = str.find("*");

  string substr = str.substr(0, plusPos);

  // Create our own temporaries so we don't change values if parsing is unsuccessful
  double lFrom = 0.0;
  double lStep = 0.0;
  unsigned int lNSteps = 0;

  // Try to get the from value
  bool castOk = true;
  try
  {
    lFrom = lexical_cast<double>(substr);
  }
  catch(const boost::bad_lexical_cast &)
  {
    castOk = false;
  }

  // Try to get step if it exists
  if(castOk && plusPos != string::npos)
  {
    castOk = true;
    const size_t plusEnd = timesPos == string::npos ? string::npos : timesPos - 1;
    try
    {
      substr = str.substr(plusPos + 1, plusEnd - plusPos);
      lStep = lexical_cast<double>(substr);
    }
    catch(const boost::bad_lexical_cast &)
    {
      castOk = false;
    }

    // Try to get nsteps if it exists
    if(castOk && timesPos != string::npos)
    {
      castOk = true;
      try
      {
        substr = str.substr(timesPos + 1, string::npos);
        lNSteps = lexical_cast<unsigned int>(substr);
      }
      catch(const boost::bad_lexical_cast &)
      {
        castOk = false;
      }
    }
  }

  // Copy over temporaries if successful
  if(castOk)
  {
    from = lFrom;
    step = lStep;
    nSteps = lNSteps;
  }

  return castOk;
}

}}
