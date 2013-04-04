/*
 * CellReaderWriter.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "io/CellReaderWriter.h"

#include <boost/algorithm/string/find.hpp>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/regex.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/tokenizer.hpp>

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

::std::vector<std::string> CellReaderWriter::getSupportedFileExtensions() const
{
  return ::std::vector< ::std::string>(1, "cell");
}

common::types::StructurePtr CellReaderWriter::readStructure(
  const ResourceLocator & locator,
	const ::sstbx::common::AtomSpeciesDatabase & speciesDb) const
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

size_t CellReaderWriter::readStructures(
  StructuresContainer & outStructures,
	const ResourceLocator & resourceLocator,
	const common::AtomSpeciesDatabase & speciesDb) const
{
  common::types::StructurePtr structure = readStructure(resourceLocator, speciesDb);
  if(structure.get())
  {
    outStructures.push_back(structure.release());
    return 1;
  }
  return 0;
}

common::types::StructurePtr CellReaderWriter::readStructure(
  ::std::istream & is,
	const ::sstbx::common::AtomSpeciesDatabase & speciesDb
) const
{
  static const ::boost::regex RE_FLOAT("([-+]?[0-9]*\\.?[0-9]+([eE][-+]?[0-9]+)?)");
  typedef boost::tokenizer<boost::char_separator<char> > Tok;
  const boost::char_separator<char> sep(" \t");

  common::types::StructurePtr structure(new common::Structure());

  ::std::string line;
  ::boost::smatch match;
  if(findFirstLine(line, is, "%BLOCK lattice_", false))
  {
    // Unit cell
    if(::boost::ifind_first(line, "_abc"))
    { // abc format
      while(::std::getline(is, line) && !::boost::ifind_first(line, "%ENDBLOCK"))
      {
        if(::boost::regex_search(line, match, RE_FLOAT)) // Have we reached numbers yet?
        {
          // TODO
        }
      }
    }
    else if(::boost::ifind_first(line, "_cart"))
    { // cart format
      int vec = 0;
      ::arma::mat33 mtx;
      while(::std::getline(is, line) && !::boost::ifind_first(line, "%ENDBLOCK"))
      {
        if(::boost::regex_search(line, match, RE_FLOAT)) // Have we reached numbers yet?
        {
          Tok tok(line, sep);
          Tok::iterator tokIt = tok.begin();
          mtx(0, vec) = ::boost::lexical_cast<double>(*tokIt);
          mtx(1, vec) = ::boost::lexical_cast<double>(*++tokIt);
          mtx(2, vec) = ::boost::lexical_cast<double>(*++tokIt);

          if(++vec == 3)
            break;
        }
      }
      if(vec == 3)
        structure->setUnitCell(makeUniquePtr(new common::UnitCell(mtx)));
    }
  }

  // Reset the stream
  is.clear(); // Clear the EoF flag
  is.seekg(0, is.beg);

  const common::UnitCell * const unitCell = structure->getUnitCell();
  if(!unitCell)
    return common::types::StructurePtr();

  if(findFirstLine(line, is, "%BLOCK positions_", false))
  {
    common::AtomSpeciesId species;
    ::arma::vec3 pos;
    bool found;
    if(::boost::ifind_first(line, "_frac"))
    { // fractional format
      while(::std::getline(is, line) && !::boost::ifind_first(line, "%ENDBLOCK"))
      {
        found = true;
        Tok tok(line, sep);
        Tok::iterator tokIt = tok.begin();
        species = speciesDb.getIdFromSymbol(*tokIt);
        found &= species != common::AtomSpeciesId::DUMMY;

        pos(0) = ::boost::lexical_cast<double>(*++tokIt);
        pos(1) = ::boost::lexical_cast<double>(*++tokIt);
        pos(2) = ::boost::lexical_cast<double>(*++tokIt);

        unitCell->cartToFracInplace(pos);

        if(found)
          structure->newAtom(species).setPosition(pos);
      }
    }
    else if(::boost::ifind_first(line, "_abs"))
    { // cart format
      // TODO
    }
  }


  return structure;
}

void CellReaderWriter::writeStructure(
	common::Structure & structure,
	const ResourceLocator & locator,
	const common::AtomSpeciesDatabase & speciesDb) const
{
  const fs::path filepath(locator.path());
	if(!filepath.has_filename())
    return; // Can't write out structure without filepath

  const fs::path dir(filepath.parent_path());
	if(!dir.empty() && !exists(dir))
		create_directories(dir);

  fs::ofstream strFile;
	strFile.open(filepath);

  writeStructure(strFile, structure, speciesDb);

 if(strFile.is_open())
    strFile.close();
}

void CellReaderWriter::writeStructure(
  ::std::ostream & os,
  common::Structure & structure,
  const common::AtomSpeciesDatabase & speciesDb
) const
{
  using namespace utility::cell_params_enum;

  const common::UnitCell * unitCell = structure.getUnitCell();

  boost::scoped_ptr<common::UnitCell> boundingCell;
  if(!unitCell)
  {
    double latticeParams[6];
    // TODO: CASTEP needs a cell, so create one
    latticeParams[A] = 10;
    latticeParams[B] = 10;
    latticeParams[C] = 10;
    latticeParams[ALPHA] = 90.0;
    latticeParams[BETA] = 90.0;
    latticeParams[GAMMA] = 90.0;
    boundingCell.reset(new common::UnitCell(latticeParams));
    unitCell = boundingCell.get();
  }

  writeLatticeBlock(os, *unitCell);
  os << ::std::endl;
  writePositionsBlock(os, structure, *unitCell, speciesDb);
}

void CellReaderWriter::writeLatticeBlock(::std::ostream & os, const common::UnitCell & unitCell) const
{
  using namespace utility::cell_params_enum;

  const double (&params)[6] = unitCell.getLatticeParams();
  os << "%BLOCK LATTICE_ABC" << ::std::endl;
  os << params[A] << " " << params[B] << " " << params[C] << ::std::endl;
  os << params[ALPHA] << " " << params[BETA] << " " << params[GAMMA] << ::std::endl;
  os << "%ENDBLOCK LATTICE_ABC" << ::std::endl;
}

void CellReaderWriter::writePositionsBlock(
  ::std::ostream & os,
  const common::Structure & structure,
  const common::UnitCell & unitCell,
  const common::AtomSpeciesDatabase & speciesDb
) const
{
  using namespace utility::cart_coords_enum;

  ::arma::mat positions;
  structure.getAtomPositions(positions);
  unitCell.cartsToFracInplace(positions);
  unitCell.wrapVecsFracInplace(positions);
  os << "%BLOCK POSITIONS_FRAC" << ::std::endl;
  for(size_t i = 0; i < structure.getNumAtoms(); ++i)
  {
    const common::Atom & atom = structure.getAtom(i);
    os << *speciesDb.getSymbol(atom.getSpecies()) << " ";
    os << positions(X, i) << " " << positions(Y, i) << " " << positions(Z, i) << ::std::endl;    
  }
  os << "%ENDBLOCK POSITIONS_FRAC" << ::std::endl;
}



}
}
