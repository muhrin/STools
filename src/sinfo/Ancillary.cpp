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

// stools_common includes
#include <utility/CustomTokens.h>
#include <utility/StringParsing.h>
#include <utility/TerminalFunctions.h>

namespace stools {
namespace sinfo {

// NAMESPACES ////////////////////////////////
namespace fs = ::boost::filesystem;
namespace po = ::boost::program_options;
namespace ssc = ::spl::common;
namespace structure_properties = ssc::structure_properties;

const ::std::string VAR_BRACKET("$");
const ::std::string VAR_FORMAT("%");
const ::std::string VAR_TITLE("@");
const ::std::string DEFAULT_EMPTY_STRING("n/a");
const ::std::string DEFAULT_INFO_STRING = "$n$ $p$ $va$ $ha$ $rha$ $sg$ $tf$ $fo$ ";
const ::std::string INFO_STRING_AUTO = "auto";
const char ADDITIONAL_TOKEN_PREFIX = '+';
const double MAX_HULL_DIST_IGNORE = -1.0;

::std::string
mapEnvToOptionName(const ::std::string & envVariable)
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
  ::std::vector< ::std::string> DEFAULT_INPUT_FILES(1, fs::current_path().string());

  ::std::stringstream tokensDescription;
  for(TokensMap::const_iterator it = tokensMap.begin(), end = tokensMap.end(); it != end; ++it)
  {
    tokensDescription << it->first << "\t= " << it->second->getName();
    if(!it->second->getDefaultFormatString().empty())
      tokensDescription << " (" << it->second->getDefaultFormatString() << ")";
    tokensDescription << ::std::endl;
  }

  try
  {
    po::options_description desc(
        "Usage: " + exeName + " [options] files...\n" + tokensDescription.str() + "\nOptions");
    desc.add_options()("help", "Show help message")("info-string,i",
        po::value< ::std::string>(&in.infoString)->default_value(INFO_STRING_AUTO),
        "info string, auto = automatically generate based on input")("key,k",
        po::value< ::std::string>(&in.sortToken)->default_value("rha"), "sort token")("reverse,R",
        po::value< bool>(&in.reverseSortComparison)->default_value(false)->zero_tokens(),
        "reverse sort order")("empty,e",
        po::value< ::std::string>(&in.emptyString)->default_value(DEFAULT_EMPTY_STRING),
        "empty string - used when a value is not found")("free-mode,f",
        po::value< bool>(&in.freeMode)->default_value(false)->zero_tokens(),
        "use free mode, input string will not be automatically parsed into columns")("formula,F",
        po::value< ::std::string>(&in.filterString)->default_value(""),
        "list only structures that have a this formula or multipler there of")("no-header,n",
        po::value< bool>(&in.noHeader)->default_value(false)->zero_tokens(),
        "don't print column header")("recursive,r",
        po::value< bool>(&in.recursive)->default_value(false)->zero_tokens(),
        "descend into directories recursively")("summary,s",
        po::value< bool>(&in.summary)->default_value(false)->zero_tokens(), "summary only")("top,t",
        po::value< int>(&in.printTop)->default_value(PRINT_ALL), "print top n structures")(
        "input-file", po::value< ::std::vector< ::std::string> >(&in.inputFiles), "input file(s)")(
        "unique,u", po::value< bool>(&in.uniqueMode)->default_value(false)->zero_tokens(),
        "use only unique structures")("unique-tol,T",
        po::value< double>(&in.uniqueTolerance)->default_value(0.001),
        "tolernace to use when comparing unique structures")("composition-top",
        po::value< int>(&in.compositionTop)->default_value(0),
        "keep only the top n of each composition")
#ifdef SPL_WITH_CGAL
        ("max-hull-dist",
        po::value< double>(&in.maxHullDist)->default_value(MAX_HULL_DIST_IGNORE),
        "only print structures that lie below this distance above the hull")
        ("stable-compositions,h",
        po::value< bool>(&in.stableCompositions)->default_value(false)->zero_tokens(),
        "use convex hull to get only the stable compositions")
#endif
        ; // Need this semi-colon to finish the list of input options

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
  if(stools::utility::isStdInPipedOrFile())
  {
    while(std::cin >> lineInput)
    {
      in.inputFiles.push_back(lineInput);
    }
  }

  if(in.summary && in.printTop == -1)
    in.printTop = 1;

  if(in.inputFiles.empty())
    in.inputFiles = DEFAULT_INPUT_FILES;
  else
    parseAdditionalTokens(in);

  // Everything we okay, so clean up the input
  utility::replaceControlSequences(in.infoString);

  return Result::SUCCESS;
}

void
parseAdditionalTokens(InputOptions & in)
{
  for(::std::vector< ::std::string>::iterator it = in.inputFiles.begin(); it != in.inputFiles.end();
      /* increment in loop */)
  {
    if(!it->empty() && (*it)[0] == ADDITIONAL_TOKEN_PREFIX)
    {
      // Extract the additional token
      if(it->size() > 1)
        in.additionalTokens.push_back(::std::string(it->begin() + 1, it->end()));

      it = in.inputFiles.erase(it);
    }
    else
      ++it;
  }
}

void
addToken(TokensMap & map, TokenPtr token)
{
  // WARNING: Have to store reference to symbol as if we passed 'token->getSymbol()' directly
  // to insert then right-to-left parameter evaluation would pass ownership on before we could
  // make the call
  const ::std::string & symbol = token->getSymbol();
  map.insert(symbol, token);
}

CustomisableTokens
generateTokens(TokensMap & map)
{
  typedef utility::RelativeValueToken< double> RelativeValue;
  typedef ::std::auto_ptr< RelativeValue> RelativeValueTokenPtr;

  CustomisableTokens customisable;

  RelativeValueTokenPtr lowestEnergy(
      new RelativeValue("dU", "ru", structure_properties::general::ENERGY_INTERNAL, "%.4f"));
  RelativeValueTokenPtr lowestEnergyPerAtom(
      new RelativeValue("dU/atom", "rua", structure_properties::general::ENERGY_INTERNAL, "%.4f",
          true // Per atom
          ));
  RelativeValueTokenPtr lowestEnthalpy(
      new RelativeValue("dH", "rh", structure_properties::general::ENTHALPY, "%.4f"));
  RelativeValueTokenPtr lowestEnthalpyPerAtom(
      new RelativeValue("dH/atom", "rha", structure_properties::general::ENTHALPY, "%.4f", true // Per atom
          ));

  // Leave behind non-owning observers
  customisable.lowestEnergy = lowestEnergy.get();
  customisable.lowestEnergyPerAtom = lowestEnergyPerAtom.get();
  customisable.lowestEnthalpy = lowestEnthalpy.get();
  customisable.lowestEnthalpyPerAtom = lowestEnthalpyPerAtom.get();

  // And place in the map
  addToken(map, lowestEnergy.operator ::std::auto_ptr<utility::InfoToken>());
  addToken(map, lowestEnergyPerAtom.operator ::std::auto_ptr<utility::InfoToken>());
  addToken(map, lowestEnthalpy.operator ::std::auto_ptr<utility::InfoToken>());
  addToken(map, lowestEnthalpyPerAtom.operator ::std::auto_ptr<utility::InfoToken>());

  // Rest of the tokens

  addToken(map,
      utility::makeFunctionToken< ::std::string>("Name", "n", utility::functions::getName, "%|-|"));
  addToken(map, TokenPtr(new utility::FormulaToken("Formula", "fo", "%|-|")));
  addToken(map, TokenPtr(new utility::FormulaToken("Formula", "rfo", "%|-|", true)));

  addToken(map,
      utility::makeFunctionToken< double>("V", "v", utility::functions::getVolume, "%.2f"));
  addToken(map,
      utility::makeFunctionToken< double>("V/atom", "va", utility::functions::getVolumePerAtom,
          "%.2f"));
  addToken(map,
      TokenPtr(
          new RelativeValue("U/atom", "ua", structure_properties::general::ENERGY_INTERNAL, 0.0,
              "%.4f", true)));
  addToken(map,
      TokenPtr(
          new RelativeValue("H/atom", "ha", structure_properties::general::ENTHALPY, 0.0, "%.4f",
              true)));
  addToken(map,
      utility::makeFunctionToken< unsigned int>("N atoms", "na", utility::functions::getNumAtoms));

  addToken(map,
      utility::makeFunctionToken< ::std::string>("Spgroup", "sg",
          utility::functions::getSpaceGroupSymbol, "%|-|"));
  addToken(map,
      utility::makeFunctionToken< unsigned int>("Spgroup no.", "sgn",
          utility::functions::getSpaceGroupNumber, "%|-|"));

  addToken(map,
      utility::makeStructurePropertyToken("U", "u", structure_properties::general::ENERGY_INTERNAL,
          "%.4f"));
  addToken(map,
      utility::makeStructurePropertyToken("H", "h", structure_properties::general::ENTHALPY,
          "%.4f"));
  addToken(map,
      utility::makeStructurePropertyToken("Hf", "hf",
          structure_properties::general::FORMATION_ENTHALPY, "%.4f"));
  addToken(map,
      utility::makeStructurePropertyToken("Hull dist", "hd",
          structure_properties::general::HULL_DISTANCE, "%.4f"));
  addToken(map,
      utility::makeStructurePropertyToken("P", "p",
          structure_properties::general::PRESSURE_INTERNAL, "%.4f"));
  addToken(map,
      utility::makeStructurePropertyToken("x found", "tf",
          structure_properties::searching::TIMES_FOUND));

  addToken(map,
      utility::makeFunctionToken< ::spl::io::ResourceLocator>("File", "f",
          utility::functions::getRelativeLoadPath, "%|-|"));

  return customisable;
}

::std::string
parseTokenNames(const TokensMap & tokensMap, const InputOptions & in)
{
  typedef ::boost::find_iterator< ::std::string::iterator> StringFindIterator;
  static const ::boost::regex RE_MATCH_TITLES(VAR_TITLE + "[^[:space:]]+" + VAR_TITLE);

  ::std::string namesSubstituted = in.infoString;

  ::std::string token;
  for(StringFindIterator it = ::boost::make_find_iterator(namesSubstituted,
      ::boost::algorithm::regex_finder(RE_MATCH_TITLES)), end = StringFindIterator(); it != end;
      ++it)
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

void
addFormatString(TokensInfo & tokensInfo, const ::std::string & formatString,
    const InputOptions & in)
{
  if(in.freeMode)
    tokensInfo.formatStrings[0] += formatString;
  else
    tokensInfo.formatStrings.push_back(formatString);
}

Result::Value
getRequiredTokens(TokensInfo & tokensInfo, const TokensMap & tokensMap, const InputOptions & in)
{
  typedef ::boost::tokenizer< ::boost::char_separator< char> > Tok;
  const ::boost::char_separator< char> sep(VAR_BRACKET.c_str(), "", ::boost::keep_empty_tokens);

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
  for(Tok::const_iterator it = tokens.begin(), end = tokens.end(); it != end; ++it)
  {
    // Check for presence of format string
    entry = *it;
    if(atToken)
    {
      const size_t formatPos = entry.find(VAR_FORMAT);
      if(formatPos != ::std::string::npos)
      {
        const ::std::string tokString = entry.substr(0, formatPos);
        const TokensMap::const_iterator tokenIt = tokensMap.find(tokString);
        if(tokenIt != tokensMap.end())
        {
          const ::std::string formatString(entry.substr(formatPos));
          try
          {
            formatter.parse(formatString);
            tokensInfo.tokens.push_back(tokenIt->second);
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
        const TokensMap::const_iterator tokenIt = tokensMap.find(entry);
        if(tokenIt != tokensMap.end())
        {
          const ::std::string formatString =
              tokenIt->second->getDefaultFormatString().empty() ?
                  VAR_FORMAT + "||" : tokenIt->second->getDefaultFormatString();
          tokensInfo.tokens.push_back(tokenIt->second);
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
    const TokensMap::const_iterator it = tokensMap.find(in.sortToken);
    if(it != tokensMap.end())
      tokensInfo.sortToken = it->second;
    else
      ::std::cerr << "Unrecognised sort token: " << in.sortToken << ::std::endl;
  }

  return Result::SUCCESS;
}

void
printInfoFreeMode(const StructureInfoTable & infoTable, const SortedKeys & structures,
    const TokensInfo & tokensInfo, const InputOptions & in, const size_t numToPrint)
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
    BOOST_FOREACH(const utility::InfoToken * token, tokensInfo.tokens)
    {
      if(!token->getColumn().feedFormatter(formatter, infoTable, structures[row]))
        formatter % in.emptyString;
    }
    ::std::cout << formatter;
  }
}

void
printInfoColumnMode(const StructureInfoTable & infoTable, const SortedKeys & structures,
    const TokensInfo & tokensInfo, const InputOptions & in, const size_t numToPrint)
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

  const size_t numColumns = tokensInfo.tokens.size();

  ::std::vector< size_t> maxWidths(numColumns);

  for(size_t col = 0; col < numColumns; ++col)
  {
    // Parse the new format string for this column
    formatter.parse(tokensInfo.formatStrings[col]);

    const utility::InfoToken & token = *tokensInfo.tokens[col];

    BOOST_FOREACH(const ssc::Structure * const structure, structures)
    {
      // Get the value
      if(!token.getColumn().feedFormatter(formatter, infoTable, structure))
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
      colName = tokensInfo.tokens[col]->getName();

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
      if(!tokensInfo.tokens[col]->getColumn().feedFormatter(formatter, infoTable, structures[row]))
        formatter % in.emptyString;
      ::std::cout << formatter << " ";
    }
    ::std::cout << ::std::endl;
  }
}

void
printInfo(const StructureInfoTable & infoTable, const SortedKeys & orderedKeys,
    const TokensInfo & tokensInfo, const InputOptions & in,
    const size_t numToPrint)
{
  if(tokensInfo.tokens.empty())
    return;
  if(infoTable.size() == 0)
    return;

  if(in.freeMode)
    printInfoFreeMode(infoTable, orderedKeys, tokensInfo, in, numToPrint);
  else
    printInfoColumnMode(infoTable, orderedKeys, tokensInfo, in, numToPrint);

  if(in.summary)
  {
    ::std::cout << "Total structures: " << infoTable.size() << ::std::endl;
  }
}

}
}
