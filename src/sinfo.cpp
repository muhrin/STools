/*
 * sinfo.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////

#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string_regex.hpp> 
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/program_options.hpp>
#include <boost/ptr_container/ptr_map.hpp>
#include <boost/regex.hpp>

//#include <armaillo>

// From Pipelib //


// From SSLib //
#include <common/AtomSpeciesDatabase.h>
#include <common/Structure.h>
#include <common/StructureProperties.h>
#include <common/Types.h>
#include <common/UnitCell.h>
#include <io/ResReaderWriter.h>
#include <utility/TypedDataTable.h>

// My includes //
#include "utility/CustomTokens.h"
#include "utility/InfoToken.h"

// NAMESPACES ////////////////////////////////
namespace fs = ::boost::filesystem;
namespace po = ::boost::program_options;
namespace ssu = ::sstbx::utility;
namespace ssc = ::sstbx::common;
namespace structure_properties = ssc::structure_properties;
namespace stu = ::stools::utility;

// CLASSES //////////////////////////////////////
struct InputOptions
{
  ::std::vector< ::std::string> inputFiles;
  ::std::string infoString;
  ::std::string sortToken;
};



// TYPEDEFS /////////////////////////////////////
typedef ssu::TypedDataTable<const ssc::Structure *> StructureInfoTable;
typedef StructureInfoTable::SortedKeys SortedKeys;
typedef ::boost::ptr_map< ::std::string, stools::utility::InfoToken> TokensMap;
typedef ::std::vector< ::std::string> InfoStringTokens;

// CONSTATNS ////////////////////////////////////
const ::std::string VAR_PREFIX("%");

// FORWARD DECLARES ////////////////////////////
int processInputOptions(InputOptions & in, const int argc, const char * const argv[], const TokensMap & tokensMap);
void printInfo(
  const SortedKeys & keysOrder,
  const StructureInfoTable & infoTable,
  const TokensMap & tokensMap,
  const ::std::string & infoString,
  const InfoStringTokens & tokens
);
void generateTokens(TokensMap & map);
void addToken(TokensMap & map, ::std::auto_ptr<stu::InfoToken> token);
int parseInfoString(InfoStringTokens & infoStringTokens, const ::std::string & infoString, const TokensMap & tokensMap);

int main(const int argc, char * argv[])
{
  // Set up the tokens that we know about
  TokensMap tokensMap;
  generateTokens(tokensMap);

  // Process input and detect errors
  InputOptions in;
  int result = processInputOptions(in, argc, argv, tokensMap);
  if(result != 0)
    return result;

  // Now parse the info line string
  InfoStringTokens infoStringTokens;
  result = parseInfoString(infoStringTokens, in.infoString, tokensMap);
  if(result != 0)
    return result;

  StructureInfoTable infoTable;

  sstbx::io::ResReaderWriter resReader;

  SortedKeys sortedKeys;

  ssc::AtomSpeciesDatabase speciesDb;
  ::std::string inputFile;
  fs::path structurePath;
  ssc::StructurePtr structure;
  BOOST_FOREACH(inputFile, in.inputFiles)
  {
    fs::path structurePath(inputFile);
    structure = resReader.readStructure(structurePath, speciesDb);

    if(structure.get())
    {
      sortedKeys.push_back(structure.get());
      BOOST_FOREACH(const ::std::string & tokenEntry, infoStringTokens)
      {
        tokensMap.at(tokenEntry).insert(infoTable, *structure);
      }
    }
  }

  if(!in.sortToken.empty())
  {
    const TokensMap::const_iterator it = tokensMap.find(in.sortToken);
    if(it != tokensMap.end())
    {
      it->second->sort(sortedKeys, infoTable);
    }
  }


  printInfo(sortedKeys, infoTable, tokensMap, in.infoString, infoStringTokens);

  return 0;
}

int processInputOptions(InputOptions & in, const int argc, const char * const argv[], const TokensMap & tokensMap)
{
  const ::std::string exeName(argv[0]);

  ::std::stringstream tokensDescription;
  BOOST_FOREACH(const TokensMap::const_reference token, tokensMap)
  {
    tokensDescription << VAR_PREFIX << token.first << "\t= " << token.second->getName() << ::std::endl;
  }

  try
  {
    po::options_description desc("Usage: " + exeName + " [options] files...\n" +
      tokensDescription.str() +
      "\nOptions:");
    desc.add_options()
      ("help", "Show help message")
      ("info-string,i", po::value< ::std::string>(&in.infoString)->default_value("%n\t %p\t %v\t %e\t %sg\t %tf \n"), "info string")
      ("sort,s", po::value< ::std::string>(&in.sortToken), "sort token")
      ("input-file", po::value< ::std::vector< ::std::string> >(&in.inputFiles)->required(), "input file(s)")
    ;

    po::positional_options_description p;
    p.add("input-file", -1);

    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).options(desc).positional(p).run(), vm);

    // Deal with help first, otherwise missing required parameters will cause exception
    if(vm.count("help"))
    {
      ::std::cout << desc << ::std::endl;
      return 1;
    }

    po::notify(vm);
  }
  catch(std::exception& e)
  {
    ::std::cerr << e.what() << ::std::endl;
    return 1;
  }

  return 0;
}

void generateTokens(TokensMap & map)
{
  typedef ::std::auto_ptr<stu::InfoToken> TokenPtr;

  addToken(map, stu::makeFunctionToken< ::std::string>("Name", "n", stu::functions::getName));
  addToken(map, stu::makeFunctionToken<double>("Volume", "v", stu::functions::getVolume));
  addToken(map, stu::makeFunctionToken<double>("Energy/atom", "ea", stu::functions::getEnergyPerAtom));
  addToken(map, stu::makeFunctionToken<unsigned int>("N atoms", "na", stu::functions::getNumAtoms));
  addToken(map, stu::makeStructurePropertyToken("Spgroup", "sg", structure_properties::general::SPACEGROUP_SYMBOL));
  addToken(map, stu::makeStructurePropertyToken("Spgroup no.", "sgn", structure_properties::general::SPACEGROUP_NUMBER));
  addToken(map, stu::makeStructurePropertyToken("Energy", "e", structure_properties::general::ENERGY_INTERNAL));
  addToken(map, stu::makeStructurePropertyToken("Pressure", "p", structure_properties::general::PRESSURE_INTERNAL));
  addToken(map, stu::makeStructurePropertyToken("Times found", "tf", structure_properties::searching::TIMES_FOUND));
}

void addToken(TokensMap & map, ::std::auto_ptr<stu::InfoToken> token)
{
  // WARNING: Have to store reference to symbol as if we passed 'token->getSymbol()' directly
  // to insert then right-to-left parameter evaluation would pass ownership on before we could
  // make the call
  const ::std::string & symbol = token->getSymbol();
  map.insert(symbol, token);
}

int parseInfoString(
  InfoStringTokens & infoStringTokens,
  const ::std::string & infoString,
  const TokensMap & tokensMap)
{
  ::std::string token;

  ::boost::regex regex(VAR_PREFIX + "[[:word:]]+");
  ::boost::find_iterator< ::std::string::const_iterator> tokIt(infoString, ::boost::regex_finder(regex));
  ::boost::find_iterator< ::std::string::const_iterator> end;

  for(; tokIt != end; ++tokIt)
  {
    token.assign(tokIt->begin() + 1, tokIt->end());
    if(tokensMap.find(token) != tokensMap.end())
      infoStringTokens.push_back(token);
    else
      ::std::cerr << "Unrecognised token: " << ::boost::copy_range< ::std::string>(*tokIt) << ::std::endl;
  }

  return 0;
}

void printInfo(
  const SortedKeys & keysOrder,
  const StructureInfoTable & infoTable,
  const TokensMap & tokensMap,
  const ::std::string & infoString,
  const InfoStringTokens & tokens)
{
  ::boost::optional< ::std::string> stringValue;
  ::std::string infoLine;
  BOOST_FOREACH(const ssc::Structure * const structure, keysOrder)
  {
    infoLine = infoString;
    BOOST_FOREACH(const ::std::string & token, tokens)
    {
      stringValue = tokensMap.at(token).getColumn().getValue(infoTable, structure);
      if(stringValue)
      {
        ::boost::algorithm::replace_all(infoLine, VAR_PREFIX + token, *stringValue);
      }
    }
    ::std::cout << infoLine;
  }

}

