/*
 * ResReaderWriter.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "spl/io/ResReaderWriter.h"

#include <iomanip>
#include <iterator>
#include <set>

#include <boost/algorithm/string.hpp>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/tokenizer.hpp>

#include <armadillo>

#include "spl/common/Atom.h"
#include "spl/common/AtomSpeciesDatabase.h"
#include "spl/common/AtomSpeciesId.h"
#include "spl/common/AtomSpeciesInfo.h"
#include "spl/common/Structure.h"
#include "spl/common/StructureProperties.h"
#include "spl/common/Types.h"
#include "spl/common/UnitCell.h"
#include "spl/io/IoFunctions.h"
#include "spl/io/BoostFilesystem.h"
#include "spl/utility/IndexingEnums.h"

// DEFINES /////////////////////////////////

// NAMESPACES ////////////////////////////////

namespace spl {
namespace io {

namespace fs = ::boost::filesystem;
namespace ssc = ::spl::common;
namespace properties = ssc::structure_properties;

const unsigned int ResReaderWriter::DIGITS_AFTER_DECIMAL = 8;

// Set up our tokenizer to split around space and tab
typedef boost::tokenizer< boost::char_separator< char> > Tok;
const boost::char_separator< char> sep(" \t");

void
ResReaderWriter::writeStructure(::spl::common::Structure & str,
    const ResourceLocator & locator) const
{
  using namespace utility::cell_params_enum;
  using namespace utility::cart_coords_enum;
  using ::spl::common::AtomSpeciesId;
  using ::std::endl;

  const double * dValue;
  const ::std::string * sValue;
  const unsigned int * uiValue;

  const fs::path filepath(locator.path());
  if(!filepath.has_filename())
    throw "Cannot write out structure without filepath";

  const fs::path dir = filepath.parent_path();
  if(!dir.empty() && !exists(dir))
  {
    create_directories(dir);
  }

  fs::ofstream strFile;
  strFile.open(filepath);

  const common::UnitCell * const cell = str.getUnitCell();

  //////////////////////////
  // Start title
  strFile << "TITL ";

  if(!str.getName().empty())
    strFile << str.getName();
  else
    strFile << filepath.stem();

  // Presssure
  strFile << " ";
  dValue = str.getProperty(properties::general::PRESSURE);
  if(dValue)
    io::writeToStream(strFile, *dValue, DIGITS_AFTER_DECIMAL);
  else
    strFile << "0.0";

  // Volume
  strFile << " ";
  if(cell)
    io::writeToStream(strFile, cell->getVolume(), DIGITS_AFTER_DECIMAL);
  else
    strFile << "0.0";

  // Enthalpy
  strFile << " ";
  dValue = str.getProperty(properties::general::ENTHALPY);
  if(!dValue)
    dValue = str.getProperty(properties::general::ENERGY_INTERNAL);
  if(dValue)
    io::writeToStream(strFile, *dValue, DIGITS_AFTER_DECIMAL);
  else
    strFile << "0.0";

  // Space group
  strFile << " 0 0 " << str.getNumAtoms() << " (";
  sValue = str.getProperty(properties::general::SPACEGROUP_SYMBOL);
  if(sValue)
    strFile << *sValue;
  else
    strFile << "P1";

  // Times found
  strFile << ") n - ";
  uiValue = str.getProperty(properties::searching::TIMES_FOUND);
  if(uiValue)
    strFile << *uiValue;
  else
    strFile << "1";

  strFile << endl;
  // End title //////////////////

  ///////////////////////////////////
  // Start lattice
  if(cell)
  {
    const double (&latticeParams)[6] = cell->getLatticeParams();

    // Do cell parameters
    strFile << "CELL 1.0";
    for(size_t i = A; i <= GAMMA; ++i)
      strFile << " " << latticeParams[i];
    strFile << endl;
  }
  strFile << "LATT -1" << endl;

  // End lattice

  ////////////////////////////
  // Start atoms

  // Get the species and positions of all atoms
  using std::vector;
  using std::set;

  ::arma::mat positions;
  str.getAtomPositions(positions);
  if(cell)
  {
    cell->cartsToFracInplace(positions);
    cell->wrapVecsFracInplace(positions);
  }

  vector< AtomSpeciesId::Value> species;
  str.getAtomSpecies(::std::back_inserter(species));
  set< AtomSpeciesId::Value> uniqueSpecies(species.begin(), species.end());

  // Output atom species
  std::map< AtomSpeciesId::Value, std::string> speciesSymbols;
  std::map< AtomSpeciesId::Value, unsigned int> speciesOrder;
  strFile << "SFAC";
  size_t idx = 1;
  BOOST_FOREACH(const AtomSpeciesId::Value id, uniqueSpecies)
  {
    speciesSymbols[id] = id.empty() ? "?" : id;
    speciesOrder[id] = idx;
    ++idx;
    strFile << " " << speciesSymbols[id];
  }

  // Now write out the atom positions along with the spcies
  for(size_t i = 0; i < positions.n_cols; ++i)
  {
    const AtomSpeciesId::Value id = species[i];

    strFile << endl << speciesSymbols[id] << " " << speciesOrder[id] << " "
        << toString(positions(X, i), DIGITS_AFTER_DECIMAL) << " "
        << toString(positions(Y, i), DIGITS_AFTER_DECIMAL) << " "
        << toString(positions(Z, i), DIGITS_AFTER_DECIMAL) << " 1.0";
  }

  // End atoms ///////////

  strFile << endl << "END" << endl;

  str.setProperty(properties::io::LAST_ABS_FILE_PATH,
      io::ResourceLocator(io::absolute(filepath)));

  if(strFile.is_open())
    strFile.close();
}

ssc::types::StructurePtr
ResReaderWriter::readStructure(const ResourceLocator & resourceLocator) const
{
  namespace utility = ::spl::utility;
  using spl::common::Atom;
  using spl::common::AtomSpeciesId;
  using std::endl;
  using std::getline;
  using boost::bad_lexical_cast;
  using boost::lexical_cast;
  using boost::filesystem::ifstream;

  const fs::path filepath = resourceLocator.path();
  if(!filepath.has_filename())
    throw "Cannot read structure without filepath";

  common::types::StructurePtr str;

  if(!exists(filepath))
    return str;

  ifstream strFile;
  strFile.open(filepath);

  if(strFile.is_open())
  {
    str.reset(new common::Structure());

    str->setProperty(properties::io::LAST_ABS_FILE_PATH,
        io::ResourceLocator(io::absolute(filepath)));

    std::string line;
    for(getline(strFile, line); strFile.good(); getline(strFile, line))
    {
      if(line.find("TITL") != ::std::string::npos)
        parseTitle(*str, line);
      else if(line.find("CELL") != ::std::string::npos)
        parseCell(*str, line);
      else if(line.find("SFAC") != ::std::string::npos)
        parseAtoms(*str, strFile, line);
    } // end for

    strFile.close();
  }

  return str;
}

size_t
ResReaderWriter::readStructures(StructuresContainer & outStructures,
    const ResourceLocator & resourceLocator) const
{
  ssc::types::StructurePtr structure = readStructure(resourceLocator);

  if(structure.get())
  {
    outStructures.push_back(structure.release());
    return 1;
  }

  return 0;
}

std::vector< std::string>
ResReaderWriter::getSupportedFileExtensions() const
{
  std::vector< std::string> ext;
  ext.push_back("res");
  return ext;
}

bool
ResReaderWriter::multiStructureSupport() const
{
  return false;
}

bool
ResReaderWriter::parseTitle(common::Structure & structure,
    const ::std::string & titleLine) const
{
  Tok tok(titleLine, sep);
  // Put the tokens into a vector
  const ::std::vector< ::std::string> titleTokens(tok.begin(), tok.end());

  if(titleTokens.empty() || titleTokens[0].find("TITL") == ::std::string::npos)
    return false;

  // If there are this many tokens or more it probably means the title
  // is in airss format
  if(titleTokens.size() >= 11)
  {
    structure.setName(titleTokens[1]);
    try
    {
      structure.setPropertyFromString(properties::general::PRESSURE,
          titleTokens[2]);
    }
    catch(const ::boost::bad_lexical_cast & /*e*/)
    {
    }
    // 3 = volume
    try
    {
      structure.setPropertyFromString(properties::general::ENTHALPY,
          titleTokens[4]);
    }
    catch(const ::boost::bad_lexical_cast & /*e*/)
    {
    }
    // 5 = spin density
    // 6 = integrated spin density
    // 7 = space group or num atoms(in new format ONLY)
    size_t newFormat = 0;
    if(!titleTokens[8].empty() && titleTokens[8][0] == '(')
      newFormat = 1;

    ::std::string iucSymbol = titleTokens[7 + newFormat];
    if(!iucSymbol.empty() && iucSymbol[0] == '(')
      iucSymbol.erase(0, 1);
    if(!iucSymbol.empty() && iucSymbol[iucSymbol.size() - 1] == ')')
      iucSymbol.erase(iucSymbol.size() - 1, 1);
    if(!iucSymbol.empty())
      structure.setProperty(properties::general::SPACEGROUP_SYMBOL, iucSymbol);

    // 8 = 'n'
    // 9 = '-'
    // 10 = times found
    if(titleTokens.size() >= 11 + newFormat)
    {
      try
      {
        structure.setPropertyFromString(properties::searching::TIMES_FOUND,
            titleTokens[10 + newFormat]);
      }
      catch(const ::boost::bad_lexical_cast & /*e*/)
      {
      }
    }
    return true;
  }
  return false;
}

bool
ResReaderWriter::parseCell(common::Structure & structure,
    const ::std::string & cellLine) const
{
  const Tok tok(cellLine, sep);
  Tok::iterator it = tok.begin(), end = tok.end();
  if(it == end || it->find("CELL") == ::std::string::npos)
    return false;

  // The cell parameters start at the 3nd entry
  if(++it == end)
    return false;
  ++it;

  bool paramsFound = true;
  double params[6] =
    { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  int i = 0;
  for(/* already initialised */; i < 6 && it != end; ++i, ++it)
  {
    try
    {
      params[i] = ::boost::lexical_cast< double>(*it);
    }
    catch(const ::boost::bad_lexical_cast & /*e*/)
    {
      paramsFound = false;
      break;
    }
  }
  if(!paramsFound || i < 6)
    return false;

  structure.setUnitCell(common::UnitCell(params));
  return true;
}

bool
ResReaderWriter::parseAtoms(common::Structure & structure,
    ::std::istream & inStream, const ::std::string & sfacLine) const
{
  using namespace utility::cart_coords_enum;

  ::std::string line;

  common::AtomSpeciesId::Value atomId;
  bool encounteredProblem = false;
  ::arma::vec3 pos;
  Tok::iterator it, end;
  while(::std::getline(inStream, line))
  {
    // Assume there is a problem unless we get to the bottom of the loop body
    Tok atomToker(line, sep);
    it = atomToker.begin();
    end = atomToker.end();

    if(it == end) // Blank line
      continue;

    if(*it == "END")
      break;

    // Try finding the species id
    atomId = *it;
    if(++it == end)
    {
      encounteredProblem = true;
      continue;
    }
    // Skip over the species index
    if(++it == end)
    {
      encounteredProblem = true;
      continue;
    }

    // Next should be the three coordinates
    try
    {
      pos(X) = ::boost::lexical_cast< double>(*it);
      if(++it == end)
      {
        encounteredProblem = true;
        continue;
      }
      pos(Y) = ::boost::lexical_cast< double>(*it);
      if(++it == end)
      {
        encounteredProblem = true;
        continue;
      }
      pos(Z) = ::boost::lexical_cast< double>(*it);
    }
    catch(const ::boost::bad_lexical_cast & /*e*/)
    {
      continue;
    }

    if(structure.getUnitCell())
      structure.getUnitCell()->fracToCartInplace(pos);
    structure.newAtom(atomId).setPosition(pos);
  }
  return !encounteredProblem;
}

void
ResReaderWriter::writeTitle(::std::ostream & os,
    const common::Structure & structure) const
{
  SSLIB_DIE_NOT_IMPLEMENTED();
}

}
}
