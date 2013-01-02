/*
 * Ancillary.cpp
 *
 *  Created on: Dec 10, 2012
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "sinfo/Ancillary.h"

#include <iomanip>

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
const ::std::string VAR_TITLE("@");
const ::std::string DEFAULT_EMPTY_STRING("n/a");
const ::std::string DEFAULT_INFO_STRING("$n%-18.18s $p%10.2f $v%10.2f $e%10.2f $re%10.2f $sg%7.7s $tf%6.6i\n");

Result::Value
processInputOptions(InputOptions & in, const int argc, char * argv[], const TokensMap & tokensMap)
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
      ("free-mode,f", po::value<bool>(&in.freeMode)->default_value(false)->zero_tokens(), "use free mode, input string will not be automatically parsed into columns")
      ("no-header,n", po::value<bool>(&in.noHeader)->default_value(false)->zero_tokens(), "don't print column header")
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

::std::string
parseTokenNames(
  const TokensMap & tokensMap,
  const InputOptions & in)
{
  typedef ::boost::find_iterator< ::std::string::iterator> StringFindIterator;
  ::boost::regex matchTitles(VAR_TITLE + "[^[:space:]]+" + VAR_TITLE);

  ::std::string namesSubstituted = in.infoString;

  ::std::string token;
  for(StringFindIterator it = ::boost::make_find_iterator(namesSubstituted, ::boost::algorithm::regex_finder(matchTitles)),
    end = StringFindIterator(); it != end; ++it)
  {
    token.assign(it->begin() + 1, it->end() - 1);
    const TokensMap::const_iterator mapIt = tokensMap.find(token);
    if(mapIt != tokensMap.end())
    {
      ::boost::replace_range(namesSubstituted, *it, mapIt->second->getName());
    }
    else
    {
      ::std::cerr << "Unrecognised token name: " << token << ::std::endl;
    }
  }

  return namesSubstituted;
}

void addFormatString(
  TokensInfo & tokensInfo,
  const ::std::string & formatString,
  const InputOptions & in
)
{
  if(in.freeMode)
    tokensInfo.formatStrings[0] += formatString;
  else
    tokensInfo.formatStrings.push_back(formatString);
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

  if(in.freeMode)
    tokensInfo.formatStrings.resize(1);
  
  const ::std::string namesSubstituted = in.freeMode ? parseTokenNames(tokensMap, in) : in.infoString;

  ::boost::format formatter;

  Tok tokens(namesSubstituted, sep);
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
            addFormatString(tokensInfo, formatString, in);
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
          addFormatString(tokensInfo, VAR_FORMAT + "||", in);
        }
        else
          ::std::cerr << "Unrecognised token: " << entry << ::std::endl;
      }
    }
    else if(in.freeMode)
      tokensInfo.formatStrings[0] += entry;

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

void printInfoFreeMode(
  const StructureInfoTable & infoTable,
  const SortedKeys & orderedKeys,
  const TokensInfo & tokensInfo,
  const TokensMap & tokensMap,
  const InputOptions & in)
{
  ::boost::format formatter;
  try
  {
    formatter.parse(tokensInfo.formatStrings[0]);
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

void printInfoColumnMode(
  const StructureInfoTable & infoTable,
  const SortedKeys & structures,
  const TokensInfo & tokensInfo,
  const TokensMap & tokensMap,
  const InputOptions & in)
{
  ::boost::format formatter;
  try
  {
    formatter.parse(tokensInfo.formatStrings[0]);
  }
  catch(const ::boost::io::bad_format_string & e)
  {
    ::std::cerr << "Error parsing format string." << ::std::endl;
    ::std::cerr << e.what() << ::std::endl;
    return;
  }

  typedef ::std::vector< ::std::string> StringsRow;
  typedef ::std::vector<StringsRow> StringsTable;

  const size_t numColumns = tokensInfo.tokenStrings.size();

  StringsTable stringsTable;
  ::std::vector<size_t> maxWidths(numColumns);

  for(size_t row = 0; row < structures.size(); ++row)
  {
    stringsTable.push_back(StringsRow(numColumns));
    for(size_t col = 0; col < numColumns; ++col)
    {
      // Parse the new format string for this column
      formatter.parse(tokensInfo.formatStrings[col]);
      
      // Get the value
      if(!tokensMap.at(tokensInfo.tokenStrings[col]).getColumn().feedFormatter(formatter, infoTable, structures[row]))
        formatter % in.emptyString;
      stringsTable[row][col] = formatter.str();
      
      // Get the max width so far
      maxWidths[col] = ::std::max(maxWidths[col], stringsTable[row][col].size());
    }
    ::std::cout << ::std::endl;
  }

  if(!in.noHeader)
  {
    ::std::string colName;
    for(size_t col = 0; col < numColumns; ++col)
    {
      // Get the column name
      colName = tokensMap.at(tokensInfo.tokenStrings[col]).getName();
      
      // Make sure we consider it for the width
      maxWidths[col] = ::std::max(maxWidths[col], colName.size());

      ::std::cout << ::std::setw(maxWidths[col] + 1) << ::std::left << colName;
    } 
  }

  // Now go through and print everything
  for(size_t row = 0; row < structures.size(); ++row)
  {
    for(size_t col = 0; col < numColumns; ++col)
    {
      ::std::cout << ::std::setw(maxWidths[col] + 1) << ::std::left << stringsTable[row][col];
    }
    ::std::cout << ::std::endl;
  }
}

void printInfo(
  const StructureInfoTable & infoTable,
  const SortedKeys & orderedKeys,
  const TokensInfo & tokensInfo,
  const TokensMap & tokensMap,
  const InputOptions & in)
{
  ::boost::format formatter;
  if(in.freeMode)
    printInfoFreeMode(infoTable, orderedKeys, tokensInfo, tokensMap, in);
  else
    printInfoColumnMode(infoTable, orderedKeys, tokensInfo, tokensMap, in);
}

}
}