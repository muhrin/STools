/*
 * UtilityFunctions.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: muhrin
 */

// INCLUDES //////////////////////////////////
#include "common/UtilityFunctions.h"

#include <iomanip>

#include <boost/functional/hash.hpp>
#include <boost/lexical_cast.hpp>

#include <spl/io/IoFunctions.h>
#include <spl/utility/Armadillo.h>

// From local
#include "common/StructureData.h"

// NAMESPACES ////////////////////////////////

namespace spipe {
namespace common {

namespace ssio = ::spl::io;

void parseParamString(
  const std::string & str,
  double &            from,
  double &            step,
  unsigned int &      nSteps
)
{
  using std::string;
  using boost::lexical_cast;

  if(str.empty())
  {
    throw ::std::invalid_argument("Cannot get parameters from empty string");
  }

  const size_t plusPos = str.find("+");
  const size_t timesPos = str.find("*");

  string substr = str.substr(0, plusPos);

  // Create our own temporaries so we don't change values if parsing is unsuccessful
  double lFrom = 0.0;
  double lStep = 0.0;
  unsigned int lNSteps = 1;

  // Try to get the from value
  try
  {
    lFrom = lexical_cast<double>(substr);
  }
  catch(const boost::bad_lexical_cast &)
  {
    throw ::std::invalid_argument("Could not parse " + substr + " as double");
  }

  // Try to get step if it exists
  if(plusPos != string::npos)
  {
    const size_t plusEnd = timesPos == string::npos ? string::npos : timesPos - 1;
    substr = str.substr(plusPos + 1, plusEnd - plusPos);
    try
    {
      lStep = lexical_cast<double>(substr);
      lNSteps = 2;  // Have two steps (the original number, lFrom, and lFrom+lStep)
    }
    catch(const boost::bad_lexical_cast &)
    {
      throw ::std::invalid_argument("Could not parse " + substr + " as double");
    }

    // Try to get nsteps if it exists
    if(timesPos != string::npos)
    {
      substr = str.substr(timesPos + 1, string::npos);
      try
      {
        lNSteps = lexical_cast<unsigned int>(substr) + 1;
      }
      catch(const boost::bad_lexical_cast &)
      {
        throw ::std::invalid_argument("Could not parse " + substr + " as unsigned integer");
      }
    }
  }

  // Copy over temporaries
  from = lFrom;
  step = lStep;
  nSteps = lNSteps;
}

::std::string toString(const double in, unsigned int digitsAfterDecimal)
{
  int digits = ssio::getPrecision(in, digitsAfterDecimal);
  ::std::ostringstream ss;
  ss << ::std::setprecision(digits) << in;
  return ss.str();
}

::std::string
generateParamDirName(const ::arma::vec & params, const ::std::string & seedName)
{
  static const ::boost::hash< ::arma::vec> VEC_HASHER = ::boost::hash< ::arma::vec>();

  std::stringstream stream;
  stream << seedName << "-" << std::hex << VEC_HASHER(params);
  return stream.str();
}

}
}
