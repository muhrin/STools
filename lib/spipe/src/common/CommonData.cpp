/*
 * SharedData.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "common/CommonData.h"

#include <boost/algorithm/string/trim.hpp>
#include <boost/lexical_cast.hpp>

// NAMESPACES ////////////////////////////////

namespace spipe {
namespace common {

namespace ssu = ::spl::utility;

ParamRange::ParamRange(const size_t dims):
from(dims),
step(dims),
nSteps(dims)
{}

bool ParamRange::fromStrings(const ::std::vector< ::std::string> & paramStrings)
{
  const size_t numParams = paramStrings.size();
  from.resize(numParams);
  step.resize(numParams);
  nSteps.resize(numParams);

  for(size_t i = 0; i < numParams; ++i)
  {
    if(!parseParamString(i, paramStrings[i]))
      return false;
  }
  return true;
}

bool ParamRange::parseParamString(const size_t idx, const ::std::string & paramString)
{
  using std::string;
  using boost::lexical_cast;

  if(paramString.empty())
    return false;

  const size_t plusPos = paramString.find("+");
  const size_t timesPos = paramString.find("*");

  string substr = paramString.substr(0, plusPos);
  ::boost::trim(substr);

  // Create our own temporaries so we don't change values if parsing is unsuccessful
  double lFrom = 0.0;
  double lStep = 0.0;
  int lNSteps = 0;

  // Try to get the from value
  try
  {
    lFrom = lexical_cast<double>(substr);
  }
  catch(const ::boost::bad_lexical_cast & /*e*/)
  {
    return false;
  }

  // Try to get step if it exists
  if(plusPos != string::npos)
  {
    // Is there only a plus?
    const size_t plusEnd = timesPos == string::npos ? string::npos : timesPos - 1;
    substr = paramString.substr(plusPos + 1, plusEnd - plusPos);
    ::boost::trim(substr);
    try
    {
      lStep = lexical_cast<double>(substr);
      lNSteps = 1;  // Have two steps (the original number, lFrom, and lFrom+lStep)
    }
    catch(const ::boost::bad_lexical_cast & /*e*/)
    {
      return false;
    }

    // Try to get nsteps if it exists
    if(timesPos != string::npos)
    {
      substr = paramString.substr(timesPos + 1, string::npos);
      ::boost::trim(substr);
      try
      {
        lNSteps = lexical_cast<int>(substr);
      }
      catch(const boost::bad_lexical_cast & /*e*/)
      {
        return false;
      }
    }
  }

  // Copy over temporaries
  from[idx] = lFrom;
  step[idx] = lStep;
  nSteps[idx] = lNSteps;
  return true;
}

// Objects keys ////////////////
ssu::Key< ::std::vector<double> > GlobalKeys::POTENTIAL_PARAMS;
ssu::Key<ParamRange> GlobalKeys::POTENTIAL_SWEEP_RANGE;


}
}
