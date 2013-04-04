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
#include <boost/lexical_cast.hpp>

#include "common/AtomSpeciesDatabase.h"
#include "common/Structure.h"
#include "common/UnitCell.h"
#include "utility/IndexingEnums.h"

// DEFINES /////////////////////////////////


// NAMESPACES ////////////////////////////////


namespace sstbx {
namespace io {

namespace fs = ::boost::filesystem;

const ::std::string CastepReader::CELL_TITLE("Unit Cell");
const ::std::string CastepReader::CONTENTS_TITLE("Cell Contents");
const ::std::string CastepReader::CONTENTS_BOX_BEGIN("x----------------------------------------------------------x");
const ::std::string CastepReader::LATTICE_PARAMS_TITLE("Lattice parameters");
const ::std::string CastepReader::RE_FLOAT("[-+]?[0-9]*\\.?[0-9]+");
const ::boost::regex CastepReader::RE_LATTICE_PARAM("[[:word:]]*[[:blank:]]*=[[:blank:]]*(" + RE_FLOAT + ")[[:blank:]]*[[:word:]]*[[:blank:]]*=[[:blank:]]*(" + RE_FLOAT + ")");
const ::boost::regex CastepReader::RE_ATOM_INFO("x[[:blank:]]*([[:word:]]+)[[:blank:]]+[[:digit:]]+[[:blank:]]+(" + RE_FLOAT + ")[[:blank:]]+(" + RE_FLOAT + ")[[:blank:]]+(" + RE_FLOAT + ")");

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

 return structure;
}

size_t CastepReader::readStructures(
  StructuresContainer & outStructures,
	const ResourceLocator & locator,
	const common::AtomSpeciesDatabase & speciesDb
) const
{
  common::types::StructurePtr structure;
  const fs::path filepath(locator.path());
	if(!filepath.has_filename())
    return 0; // Can't write out structure without filepath

  fs::ifstream strFile;
	strFile.open(filepath);

  const size_t numRead = readStructures(outStructures, strFile, speciesDb);

  if(strFile.is_open())
    strFile.close();

  return numRead;
}

::sstbx::common::types::StructurePtr CastepReader::readStructure(
  ::std::istream & inputStream,
	const ::sstbx::common::AtomSpeciesDatabase & speciesDb
) const
{
  common::types::StructurePtr structure;

  StructuresContainer container;
  if(readStructures(container, inputStream, speciesDb) > 0)
    structure.reset(container.pop_back().release());

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
  bool errorParsing = false;
  while(!errorParsing && ::std::getline(inputStream, line))
  {
    if(::boost::find_first(line, CELL_TITLE))
      errorParsing = !parseCell(currentCell, inputStream);
    else if(::boost::find_first(line, CONTENTS_TITLE))
    {
      common::types::StructurePtr structure(
        new common::Structure(makeUniquePtr(new common::UnitCell(currentCell)))
      );
      errorParsing = !parseContents(*structure, inputStream, speciesDb);
      if(!errorParsing)
        outStructures.push_back(structure.release());
    }
  }
  return outStructures.size();
}

bool CastepReader::parseCell(common::UnitCell & unitCell, ::std::istream & inputStream) const
{
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
          angle.assign(match[2].first, match[2].second);
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
        y.assign(match[3].first, match[3].second);
        z.assign(match[4].first, match[4].second);
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

}
}
