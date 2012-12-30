/*
 * Ancillary.cpp
 *
 *  Created on: Dec 10, 2012
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "sinfo/Ancillary.h"

#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string_regex.hpp> 
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/program_options.hpp>
#include <boost/regex.hpp>
#include <boost/tokenizer.hpp>

// Local includes
#include "utility/CustomTokens.h"
#include "utility/InfoToken.h"
#include "utility/StringParsing.h"
#include "utility/TerminalFunctions.h"

// NAMESPACES ////////////////////////////////

namespace stools {
namespace sinfo {

namespace fs = ::boost::filesystem;
namespace po = ::boost::program_options;
namespace ssc = ::sstbx::common;
namespace structure_properties = ssc::structure_properties;

const ::std::string VAR_BRACKET("$");
const ::std::string VAR_FORMAT("%");
const ::std::string DEFAULT_EMPTY_STRING("n/a");
const ::std::string DEFAULT_INFO_STRING("$n%-18.18s $p%10.2f $v%10.2f $e%10.2f $re%10.2f $sg%7.7s $tf%6.6i\n");

Result::Value
processInputOptions(InputOptions & in, const int argc, const char * const argv[], const TokensMap & tokensMap)
{
  const ::std::string exeName(argv[0]);

  ::std::stringstream tokensDescription;
  BOOST_FOREACH(const TokensMap::const_reference token, tokensMap)
  {
    tokensDescription << token.first << "\t= " << token.second->getName() << ::std::endl;
  }

  try
  {
    po::options_description desc("Usage: " + exeName + " [options] files...\n" +
      tokensDescription.str() +
      "\nOptions:");
    desc.add_options()
      ("help", "Show help message")
      ("info-string,s", po::value< ::std::string>(&in.infoString)->default_value(DEFAULT_INFO_STRING), "info string")
      ("key,k", po::value< ::std::string>(&in.sortToken)->default_value("re"), "sort token")
      ("empty,e", po::value< ::std::string>(&in.emptyString)->default_value(DEFAULT_EMPTY_STRING), "empty string - used when a value is not found")
      ("input-file", po::value< ::std::vector< ::std::string> >(&in.inputFiles), "input file(s)")
    ;

    po::positional_options_description p;
    p.add("input-file", -1);

    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).options(desc).positional(p).run(), vm);

    // Deal with help first, otherwise missing required parameters will cause exception
    if(vm.count("help"))
    {
      ::std::cout << desc << ::std::endl;
      return Result::FAILURE;
    }

    po::notify(vm);
  }
  catch(std::exception& e)
  {
    ::std::cerr << e.what() << ::std::endl;
    return Result::FAILURE;
  }

  // Get any input from standard in (piped)
  std::string lineInput;
  bool foundPipedInput = false;
  if(stools::utility::isStdInPipedOrFile())
  {
    while(std::cin >> lineInput)
    {
      in.inputFiles.push_back(lineInput);
      foundPipedInput = true;
    }
  }

  if(!foundPipedInput && in.inputFiles.empty())
  {
    std::cout << "No structure files given.  Supply files with piped input or as command line paramter." << std::endl;
    return Result::FAILURE;
  }

  // Everything we okay, so clean up the input
  utility::replaceControlSequences(in.infoString);

  return Result::SUCCESS;
}

void addToken(TokensMap & map, TokenPtr token)
{
  // WARNING: Have to store reference to symbol as if we passed 'token->getSymbol()' directly
  // to insert then right-to-left parameter evaluation would pass ownership on before we could
  // make the call
  const ::std::string & symbol = token->getSymbol();
  map.insert(symbol, token);
}

CustomisableTokens generateTokens(TokensMap & map)
{
  typedef ::std::auto_ptr<utility::EnergyToken> EnergyTokenPtr;

  CustomisableTokens customisable;

  EnergyTokenPtr lowestEnergy(new utility::EnergyToken("Relative energy", "re"));
  EnergyTokenPtr lowestEnergyPerAtom(new utility::EnergyToken("Relative energy/atom", "rea", true));
  // Leave behind non-owning observers
  customisable.lowestEnergy = lowestEnergy.get();
  customisable.lowestEnergyPerAtom = lowestEnergyPerAtom.get();
  // And place in the map
  addToken(map, lowestEnergy.operator ::std::auto_ptr<utility::InfoToken>());
  addToken(map, lowestEnergyPerAtom.operator ::std::auto_ptr<utility::InfoToken>());

  addToken(map, utility::makeFunctionToken< ::std::string>("Name", "n", utility::functions::getName));
  addToken(map, utility::makeFunctionToken<double>("Volume", "v", utility::functions::getVolume));
  addToken(map, TokenPtr(new utility::EnergyToken("Energy/atom", "ea", true)));
  addToken(map, utility::makeFunctionToken<unsigned int>("N atoms", "na", utility::functions::getNumAtoms));
  addToken(map, utility::makeStructurePropertyToken("Spgroup", "sg", structure_properties::general::SPACEGROUP_SYMBOL));
  addToken(map, utility::makeStructurePropertyToken("Spgroup no.", "sgn", structure_properties::general::SPACEGROUP_NUMBER));
  addToken(map, utility::makeStructurePropertyToken("Energy", "e", structure_properties::general::ENERGY_INTERNAL));
  addToken(map, utility::makeStructurePropertyToken("Pressure", "p", structure_properties::general::PRESSURE_INTERNAL));
  addToken(map, utility::makeStructurePropertyToken("Times found", "tf", structure_properties::searching::TIMES_FOUND));

  return customisable;
}

Result::Value
getRequiredTokens(
  TokensInfo & tokensInfo,
  const TokensMap & tokensMap,
  const InputOptions & in)
{
  typedef ::boost::find_iterator< ::std::string::iterator> StringFindIterator;
  typedef ::boost::tokenizer< ::boost::char_separator<char> > Tok;
  const ::boost::char_separator<char> sep(VAR_BRACKET.c_str(), "", ::boost::keep_empty_tokens);

  ::boost::format formatter;

  Tok tokens(in.infoString, sep);
  ::std::string entry;
  bool atToken = false;
  for(Tok::const_iterator it = tokens.begin(), end = tokens.end();
    it != end; ++it)
  {
    // Check for presence of format string
    entry = *it;
    if(atToken)
    {
      const size_t formatPos = entry.find(VAR_FORMAT);
      if(formatPos != ::std::string::npos)
      {
        const ::std::string tokString = entry.substr(0, formatPos);
        if(tokensMap.find(tokString) != tokensMap.end())
        {
          const ::std::string formatString(entry.substr(formatPos));
          try
          {
            formatter.parse(formatString);
            tokensInfo.tokenStrings.push_back(tokString);
            tokensInfo.formatString += formatString;
          }
          catch(::boost::io::bad_format_string & e)
          {
            ::std::cerr << e.what() << ::std::endl;
            ::std::cerr << "Skipping token: " << tokString << ::std::endl;
          }
        }
        else
          ::std::cerr << "Unrecognised token: " << entry << ::std::endl;
        atToken = !atToken;
      }
      else
      {
        // No format string, so this token MUST be a valid token
        if(tokensMap.find(entry) != tokensMap.end())
        {
          tokensInfo.tokenStrings.push_back(entry);
          tokensInfo.formatString += VAR_FORMAT + "||";
        }
        else
          ::std::cerr << "Unrecognised token: " << entry << ::std::endl;
      }
    }
    else
      tokensInfo.formatString += entry;

    atToken = !atToken;
  }

  // Check if the sort token is valid
  if(!in.sortToken.empty())
  {
    if(tokensMap.find(in.sortToken) != tokensMap.end())
      tokensInfo.sortToken = in.sortToken;
    else
      ::std::cerr << "Unrecognised sort token: " << in.sortToken << ::std::endl;
  }

  return Result::SUCCESS;
}

void printInfo(
  const StructureInfoTable & infoTable,
  const SortedKeys & orderedKeys,
  const TokensInfo & tokensInfo,
  const TokensMap & tokensMap,
  const InputOptions & in)
{
  ::boost::format formatter;
  try
  {
    formatter.parse(tokensInfo.formatString);
  }
  catch(const ::boost::io::bad_format_string & e)
  {
    ::std::cerr << "Error parsing format string." << ::std::endl;
    ::std::cerr << e.what() << ::std::endl;
    return;
  }

  BOOST_FOREACH(const ssc::Structure * const structure, orderedKeys)
  {
    BOOST_FOREACH(const ::std::string & token, tokensInfo.tokenStrings)
    {
      if(!tokensMap.at(token).getColumn().feedFormatter(formatter, infoTable, structure))
      {
        formatter % in.emptyString;
      }
    }
    ::std::cout << formatter;
  }

}

}
}
