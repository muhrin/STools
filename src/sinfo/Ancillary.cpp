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
const ::std::string DEFAULT_INFO_STRING("$n$ $p$ $v$ $e$ $re$ $sg$ $tf$\\n");

::std::string mapEnvToOptionName(const ::std::string & envVariable)
{
  if(envVariable == "SINFO_INFO_STRING")
    return "info-string";
  else if(envVariable == "SINFO_KEY")
    return "key";
  else if(envVariable == "SINFO_FREE_MODE")
    return "free-mode";
  else if(envVariable == "SINFO_NO_HEADER")
    return "no-header";

  return "";
}

Result::Value
processInputOptions(InputOptions & in, const int argc, char * argv[], const TokensMap & tokensMap)
{
  const ::std::string exeName(argv[0]);
  ::std::vector< ::std::string> DEFAULT_INPUT_FILES(1, ::boost::filesystem3::current_path().string());

  ::std::stringstream tokensDescription;
  BOOST_FOREACH(const TokensMap::const_reference token, tokensMap)
  {
    tokensDescription << token.first << "\t= " << token.second->getName();
    if(!token.second->getDefaultFormatString().empty())
      tokensDescription << " (" << token.second->getDefaultFormatString() << ")";
    tokensDescription << ::std::endl;
  }

  try
  {
    po::options_description desc("Usage: " + exeName + " [options] files...\n" +
      tokensDescription.str() +
      "\nOptions");
    desc.add_options()
      ("help", "Show help message")
      ("info-string,i", po::value< ::std::string>(&in.infoString)->default_value(DEFAULT_INFO_STRING), "info string")
      ("key,k", po::value< ::std::string>(&in.sortToken)->default_value("re"), "sort token")
      ("empty,e", po::value< ::std::string>(&in.emptyString)->default_value(DEFAULT_EMPTY_STRING), "empty string - used when a value is not found")
      ("free-mode,f", po::value<bool>(&in.freeMode)->default_value(false)->zero_tokens(), "use free mode, input string will not be automatically parsed into columns")
      ("no-header,n", po::value<bool>(&in.noHeader)->default_value(false)->zero_tokens(), "don't print column header")
      ("recursive,r", po::value<bool>(&in.recursive)->default_value(false)->zero_tokens(), "descend into directories recursively")
      ("summary,s", po::value<bool>(&in.summary)->default_value(false)->zero_tokens(), "summary only")
      ("top,t", po::value<int>(&in.printTop)->default_value(PRINT_ALL), "print top n structures")
      ("input-file", po::value< ::std::vector< ::std::string> >(&in.inputFiles), "input file(s)")
    ;

    po::positional_options_description p;
    p.add("input-file", -1);

    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).options(desc).positional(p).run(), vm);
    po::store(po::parse_environment(desc, mapEnvToOptionName), vm);

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

  if(in.summary && in.printTop == -1)
    in.printTop = 1;

  if(in.inputFiles.empty())
    in.inputFiles = DEFAULT_INPUT_FILES;

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

  EnergyTokenPtr lowestEnergy(new utility::EnergyToken("Rel. energy", "re", "%.4f"));
  EnergyTokenPtr lowestEnergyPerAtom(new utility::EnergyToken("Rel. energy/atom", "rea", "%.4f", true));
  // Leave behind non-owning observers
  customisable.lowestEnergy = lowestEnergy.get();
  customisable.lowestEnergyPerAtom = lowestEnergyPerAtom.get();
  // And place in the map
  addToken(map, lowestEnergy.operator ::std::auto_ptr<utility::InfoToken>());
  addToken(map, lowestEnergyPerAtom.operator ::std::auto_ptr<utility::InfoToken>());

  addToken(map, utility::makeFunctionToken< ::std::string>("Name", "n", utility::functions::getName, "%|-|"));
  addToken(map, utility::makeFunctionToken<double>("Volume", "v", utility::functions::getVolume, "%.2f"));
  addToken(map, TokenPtr(new utility::EnergyToken("Energy/atom", "ea", "%.4f", true)));
  addToken(map, utility::makeFunctionToken<unsigned int>("N atoms", "na", utility::functions::getNumAtoms));
  
  addToken(map, utility::makeFunctionToken< ::std::string>("Spgroup", "sg", utility::functions::getSpaceGroupSymbol, "%|-|"));
  addToken(map, utility::makeFunctionToken<unsigned int>("Spgroup no.", "sgn", utility::functions::getSpaceGroupNumber, "%|-|"));

  addToken(map, utility::makeStructurePropertyToken("Energy", "e", structure_properties::general::ENERGY_INTERNAL, "%.4f"));
  addToken(map, utility::makeStructurePropertyToken("Pressure", "p", structure_properties::general::PRESSURE_INTERNAL, "%.4f"));
  addToken(map, utility::makeStructurePropertyToken("Times found", "tf", structure_properties::searching::TIMES_FOUND));

  addToken(map, utility::makeFunctionToken< ::sstbx::io::ResourceLocator>("File", "f", utility::functions::getRelativeLoadPath, "%|-|"));

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
  
  const ::std::string namesSubstituted =
    in.freeMode ?
    parseTokenNames(tokensMap, in) :
    utility::removeVerticalPositioningSequencesCopy(parseTokenNames(tokensMap, in));

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
        const TokensMap::const_iterator mapIt = tokensMap.find(entry);
        if(mapIt != tokensMap.end())
        {
          const ::std::string formatString =
            mapIt->second->getDefaultFormatString().empty() ?
            VAR_FORMAT + "||" :
            mapIt->second->getDefaultFormatString();
          tokensInfo.tokenStrings.push_back(entry);
          addFormatString(tokensInfo, formatString, in);
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
  const SortedKeys & structures,
  const TokensInfo & tokensInfo,
  const TokensMap & tokensMap,
  const InputOptions & in,
  const size_t numToPrint)
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

  for(size_t row = 0; row < numToPrint; ++row)
  {
    BOOST_FOREACH(const ::std::string & token, tokensInfo.tokenStrings)
    {
      if(!tokensMap.at(token).getColumn().feedFormatter(formatter, infoTable, structures[row]))
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
  const InputOptions & in,
  const size_t numToPrint)
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

  const size_t numColumns = tokensInfo.tokenStrings.size();

  ::std::vector<size_t> maxWidths(numColumns);

  for(size_t row = 0; row < structures.size(); ++row)
  {
    for(size_t col = 0; col < numColumns; ++col)
    {
      // Parse the new format string for this column
      formatter.parse(tokensInfo.formatStrings[col]);
      
      // Get the value
      if(!tokensMap.at(tokensInfo.tokenStrings[col]).getColumn().feedFormatter(formatter, infoTable, structures[row]))
        formatter % in.emptyString;
      
      // Get the max width so far
      maxWidths[col] = ::std::max(maxWidths[col], formatter.str().size());
    }
  }

  if(!in.noHeader)
  {
    ::std::string colName;
    formatter.parse("%|=|");
    for(size_t col = 0; col < numColumns; ++col)
    {
      // Get the column name
      colName = tokensMap.at(tokensInfo.tokenStrings[col]).getName();
      
      // Make sure we consider it for the width
      maxWidths[col] = ::std::max(maxWidths[col], colName.size());

      formatter.modify_item(1, ::std::setw(maxWidths[col]));
      formatter % colName;
      ::std::cout << formatter << " ";
    } 
    ::std::cout << ::std::endl;
  }

  // Now go through and print everything
  for(size_t row = 0; row < numToPrint; ++row)
  {
    for(size_t col = 0; col < numColumns; ++col)
    {
      // Parse the new format string for this column
      formatter.parse(tokensInfo.formatStrings[col]).modify_item(1, ::std::setw(maxWidths[col]));
      
      // Get the value
      if(!tokensMap.at(tokensInfo.tokenStrings[col]).getColumn().feedFormatter(formatter, infoTable, structures[row]))
        formatter % in.emptyString;
      ::std::cout << formatter << " ";
    }
    ::std::cout << ::std::endl;
  }
}

void printInfo(
  const StructureInfoTable & infoTable,
  const SortedKeys & orderedKeys,
  const TokensInfo & tokensInfo,
  const TokensMap & tokensMap,
  const InputOptions & in,
  const size_t numToPrint)
{
  if(tokensInfo.tokenStrings.empty())
    return;
  if(infoTable.size() == 0)
    return;

  if(in.freeMode)
    printInfoFreeMode(infoTable, orderedKeys, tokensInfo, tokensMap, in, numToPrint);
  else
    printInfoColumnMode(infoTable, orderedKeys, tokensInfo, tokensMap, in, numToPrint);

  if(in.summary)
  {
    ::std::cout << "Total structures: " << infoTable.size() << ::std::endl;
  }
}

}
}
