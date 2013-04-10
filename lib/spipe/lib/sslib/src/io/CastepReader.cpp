/*
 * CastepReader.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "io/CastepReader.h"

#include <sstream>

#include <boost/algorithm/string/find.hpp>
#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>

#include "common/AtomSpeciesDatabase.h"
#include "common/Structure.h"
#include "common/UnitCell.h"
#include "io/BoostFilesystem.h"
#include "io/Parsing.h"
#include "utility/IndexingEnums.h"

// DEFINES /////////////////////////////////


// NAMESPACES ////////////////////////////////


namespace sstbx {
namespace io {

namespace fs = ::boost::filesystem;
namespace properties = common::structure_properties;

const ::std::string CastepReader::CELL_TITLE("Unit Cell");
const ::std::string CastepReader::CONTENTS_TITLE("Cell Contents");
const ::std::string CastepReader::CONTENTS_BOX_BEGIN("x----------------------------------------------------------x");
const ::std::string CastepReader::LATTICE_PARAMS_TITLE("Lattice parameters");

::std::vector<std::string> CastepReader::getSupportedFileExtensions() const
{
  return ::std::vector< ::std::string>(1, "castep");
}

::sstbx::common::types::StructurePtr CastepReader::readStructure(
  const ResourceLocator & locator,
	const ::sstbx::common::AtomSpeciesDatabase & speciesDb
) const
{
  common::types::StructurePtr structure;
  const fs::path filepath(locator.path());
	if(!filepath.has_filename())
    return structure; // Can't write out structure without filepath

  fs::ifstream strFile;
	strFile.open(filepath);

  structure = readStructure(strFile, speciesDb);

 if(strFile.is_open())
    strFile.close();

  // The above call will set the name of the structure to the index in the
  // .castep file.  Now, prepend the file stem to complete the name.
  if(structure.get())
    structure->setName(stemString(locator.path()) + "-" + structure->getName());

 return structure;
}

size_t CastepReader::readStructures(
  StructuresContainer & outStructures,
	const ResourceLocator & locator,
	const common::AtomSpeciesDatabase & speciesDb
) const
{
  const fs::path filepath(locator.path());
	if(!filepath.has_filename())
    return 0; // Can't write out structure without filepath

  fs::ifstream strFile;
	strFile.open(filepath);

  size_t numRead = 0;
  if(locator.id().empty()) // Get them all
    numRead = readStructures(outStructures, strFile, speciesDb);
  else
  { // Get a single structure from the identifier
    common::types::StructurePtr str(readStructure(strFile, speciesDb, locator.id()));
    if(str.get())
    {
      outStructures.push_back(str);
      numRead = 1;
    }
  }

  if(strFile.is_open())
    strFile.close();

  // The above call will set the name of the structures to the index in the
  // .castep file.  Now, prepend the file stem to complete the name.
  BOOST_FOREACH(common::Structure & structure, outStructures)
  {
    structure.setName(stemString(locator.path()) + "-" + structure.getName());
  }

  return numRead;
}

::sstbx::common::types::StructurePtr CastepReader::readStructure(
  ::std::istream & inputStream,
	const ::sstbx::common::AtomSpeciesDatabase & speciesDb,
  const ::std::string & id
) const
{
  common::types::StructurePtr structure;

  StructuresContainer container;
  // If no id or the id requests the last structure then get them all
  // and only keep the last
  if(id.empty() || id.find("last") != ::std::string::npos)
  {
    if(readStructures(container, inputStream, speciesDb) > 0)
      structure.reset(container.pop_back().release());
  }
  else
  { // The id should be an integer
    size_t structureIndex;
    try
    {
      structureIndex = ::boost::lexical_cast<size_t>(id);
      readStructures(container, inputStream, speciesDb);
      if(structureIndex < container.size())
      {
        structure.reset(
          container.release(container.begin() + structureIndex).release()
        );
      }
    }
    catch(const ::boost::bad_lexical_cast & /*e*/)
    {}
  }

  return structure;
}

size_t CastepReader::readStructures(
  StructuresContainer & outStructures,
	::std::istream & inputStream,
	const common::AtomSpeciesDatabase & speciesDb
) const
{
  common::UnitCell currentCell;
  std::string line;
  bool cellUpToDate = false;
  size_t numRead = 0;
  common::Structure * lastStructure = NULL;
  AuxInfo auxInfo;

  while(::std::getline(inputStream, line))
  {
    if(::boost::find_first(line, CELL_TITLE))
      cellUpToDate = parseCell(currentCell, inputStream);
    else if(cellUpToDate && ::boost::find_first(line, CONTENTS_TITLE))
    {
      common::types::StructurePtr structure(
        new common::Structure(makeUniquePtr(new common::UnitCell(currentCell)))
      );
      if(parseContents(*structure, inputStream, speciesDb))
      {
        // Save the last structure we loaded so we can update it with information from
        // the castep file that comes later
        lastStructure = structure.get();

        structure->setName(::boost::lexical_cast< ::std::string>(numRead));
        outStructures.push_back(structure.release());
        ++numRead;
      }
    }
    else if(lastStructure && parseAuxInfo(auxInfo, inputStream, line))
    {
      // If there is new auxiliary info then update the structure with it
      updateStructure(*lastStructure, auxInfo);
    }
  }
  return numRead;
}

bool CastepReader::parseCell(common::UnitCell & unitCell, ::std::istream & inputStream) const
{
  static const ::boost::regex RE_LATTICE_PARAM("[a|b|c][[:blank:]]*=[[:blank:]]*(" + io::PATTERN_FLOAT + ")[[:blank:]]+[[:alpha:]]+[[:blank:]]*=[[:blank:]]*(" + io::PATTERN_FLOAT + ")");

  double latticeParams[6];
  std::string line;
  bool gotParams = false, foundParams = false;;
  while(!foundParams && ::std::getline(inputStream, line))
  {
    if(::boost::find_first(line, LATTICE_PARAMS_TITLE))
    { // Lattice parameters should be on the following three
      foundParams = true;

      ::boost::smatch match;
      ::std::string param, angle;

      gotParams = true;
      for(size_t i = 0; i < 3 && gotParams; ++i)
      {
        if(::std::getline(inputStream, line) &&
          ::boost::regex_search(line, match, RE_LATTICE_PARAM)) // Get 'a = ' line
        {
          param.assign(match[1].first, match[1].second);
          angle.assign(match[3].first, match[3].second);
          try
          {
            latticeParams[i] = ::boost::lexical_cast<double>(param);
            latticeParams[i + 3] = ::boost::lexical_cast<double>(angle);
          }
          catch(const ::boost::bad_lexical_cast & /*e*/)
          {
            gotParams = false;
          }
        }
        else
          gotParams = false;
      }
    }
  }

  if(gotParams)
    unitCell.setLatticeParams(latticeParams);
  
  return gotParams;
}

bool CastepReader::parseContents(
  common::Structure & structure,
  ::std::istream & inputStream,
  const common::AtomSpeciesDatabase & speciesDb
) const
{
  using namespace utility::cart_coords_enum;

  static const ::boost::regex RE_ATOM_INFO("x[[:blank:]]*([[:word:]]+)[[:blank:]]+[[:digit:]]+[[:blank:]]+(" + io::PATTERN_FLOAT + ")[[:blank:]]+(" + io::PATTERN_FLOAT + ")[[:blank:]]+(" + io::PATTERN_FLOAT + ")");

  SSLIB_ASSERT(structure.getUnitCell());

  const common::UnitCell & unitCell = *structure.getUnitCell();

  std::string line;
  bool gotAtoms = false, foundAtoms = false;
  while(!foundAtoms && ::std::getline(inputStream, line))
  {
    if(::boost::find_first(line, CONTENTS_BOX_BEGIN))
    { // Atoms info should be on the following lines
      foundAtoms = true;

      ::boost::smatch match;
      ::std::string species, x, y, z;

      common::AtomSpeciesId::Value speciesId;
      ::arma::vec3 posVec;

      gotAtoms = true;
      while(::std::getline(inputStream, line) &&
        ::boost::regex_search(line, match, RE_ATOM_INFO))
      {
        species.assign(match[1].first, match[1].second);
        x.assign(match[2].first, match[2].second);
        y.assign(match[4].first, match[4].second);
        z.assign(match[6].first, match[6].second);
        try
        {
          posVec(X) = ::boost::lexical_cast<double>(x);
          posVec(Y) = ::boost::lexical_cast<double>(y);
          posVec(Z) = ::boost::lexical_cast<double>(z);
        }
        catch(const ::boost::bad_lexical_cast & /*e*/)
        {
          gotAtoms = false;
        }
        speciesId = speciesDb.getIdFromSymbol(species);
        unitCell.fracToCartInplace(posVec);

        structure.newAtom(speciesId).setPosition(posVec);
      }
    }
  }
  
  return gotAtoms;
}

bool CastepReader::parseAuxInfo(AuxInfo & auxInfo, ::std::istream & inputStream, const ::std::string & line) const
{
  if(line.find("* Stress Tensor *") != ::std::string::npos)
    return parseStressTensorBox(auxInfo, inputStream);
  else if(line.find("Step    |   lambda    |   F.delta   |    enthalpy") != ::std::string::npos)
    return parseOptimisationTable(auxInfo, inputStream);

  return false;
}

void CastepReader::updateStructure(common::Structure & structure, const AuxInfo & auxInfo) const
{
  if(auxInfo.pressure)
    structure.setProperty(properties::general::PRESSURE_INTERNAL, *auxInfo.pressure);
  if(auxInfo.enthalpy)
    structure.setProperty(properties::general::ENTHALPY, *auxInfo.enthalpy);
}

bool CastepReader::parseStressTensorBox(AuxInfo & auxInfo, ::std::istream & inputStream) const
{
  static const ::boost::regex RE_PRESSURE("Pressure:[[:blank:]]+(" + PATTERN_FLOAT + ")");

  auxInfo.pressure.reset();

  ::std::string line;
  ::boost::smatch match;
  while(::std::getline(inputStream, line) && inBox(line))
  {
    if(::boost::regex_search(line, match, RE_PRESSURE))
    {
      try
      {
        const ::std::string pressureString(match[1].first, match[1].second);
        auxInfo.pressure.reset(::boost::lexical_cast<double>(pressureString));
        return true;
      }
      catch(const ::boost::bad_lexical_cast & /*e*/)
      {}
      break;
    }
  }
  return false;
}

bool CastepReader::parseOptimisationTable(AuxInfo & auxInfo, ::std::istream & inputStream) const
{
  static const ::boost::regex RE_OPTIMISER_ROW("\\|[^|]+\\|[^|]+\\|[^|]+\\|[[:blank:]]+(" + PATTERN_FLOAT + ")[[:blank:]]+\\|");

  auxInfo.enthalpy.reset();

  ::std::string line;
  ::boost::smatch match;
  // Keep going around reading in the enthalpy
  while(::std::getline(inputStream, line) && inBox(line))
  {
    if(::boost::regex_search(line, match, RE_OPTIMISER_ROW))
    {
      try
      {
        const ::std::string enthalpyString(match[1].first, match[1].second);
        auxInfo.enthalpy.reset(::boost::lexical_cast<double>(enthalpyString));
      }
      catch(const ::boost::bad_lexical_cast & /*e*/)
      {}
    }
  }
  return auxInfo.enthalpy.is_initialized();
}

bool CastepReader::inBox(const ::std::string & line) const
{
  // Find the first non-whitespace character
  char firstNonBlank = ' ';
  for(size_t i = 0; i < line.size(); ++i)
  {
    if(::std::isspace(line[i]) == 0)
    {
      firstNonBlank = line[i];
      break;
    }
  }

  if(firstNonBlank == ' ')
    return false;

  if(firstNonBlank == '*')
    return true;
  else if(firstNonBlank == 'x')
    return true;
  else if (firstNonBlank == '|')
    return true;
  else if (firstNonBlank == '+')
    return true;

  return false;
}

}
}
