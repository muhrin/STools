/*
 * XyzReaderWriter.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "io/XyzReaderWriter.h"

#include <iomanip>
#include <set>
#include <vector>

#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/tokenizer.hpp>

#include <armadillo>

#include "common/Atom.h"
#include "common/AtomSpeciesDatabase.h"
#include "common/AtomSpeciesId.h"
#include "common/AtomSpeciesInfo.h"
#include "common/Structure.h"
#include "common/StructureProperties.h"
#include "common/Types.h"
#include "common/UnitCell.h"
#include "io/IoFunctions.h"
#include "io/BoostFilesystem.h"
#include "utility/IndexingEnums.h"

// DEFINES /////////////////////////////////


// NAMESPACES ////////////////////////////////


namespace sstbx {
namespace io {

namespace fs = ::boost::filesystem;
namespace ssc = ::sstbx::common;
namespace properties = ssc::structure_properties;

const unsigned int XyzReaderWriter::DIGITS_AFTER_DECIMAL = 8;

void XyzReaderWriter::writeStructure(
	::sstbx::common::Structure & str,
	const ResourceLocator & locator,
	const ::sstbx::common::AtomSpeciesDatabase & speciesDb) const
{
  using namespace utility::cell_params_enum;
  using namespace utility::cart_coords_enum;
	using ::sstbx::common::AtomSpeciesId;
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

  // Number of atoms
  strFile << str.getNumAtoms() << ::std::endl;

	//////////////////////////
	// Start Title
	if(!str.getName().empty())
		strFile << str.getName();
	else
		strFile << filepath.stem();
	
	// Presssure
  strFile << " ";
  dValue = str.getProperty(properties::general::PRESSURE_INTERNAL);
	if(dValue)
    io::writeToStream(strFile, *dValue, DIGITS_AFTER_DECIMAL);
	else
		strFile << "n/a";

	// Volume
	strFile << " ";
  if(cell)
    io::writeToStream(strFile, cell->getVolume(), DIGITS_AFTER_DECIMAL);
  else
    strFile << "n/a";

	// Enthalpy
	strFile << " ";
  dValue = str.getProperty(properties::general::ENERGY_INTERNAL);
	if(dValue)
		io::writeToStream(strFile, *dValue, DIGITS_AFTER_DECIMAL);
	else
		strFile << "n/a";

	// Space group
	strFile << " 0 0 (";
  sValue = str.getProperty(properties::general::SPACEGROUP_SYMBOL);
	if(sValue)
		strFile << *sValue;
	else
		strFile << "n/a";

	// Times found
	strFile << ") n - ";
  uiValue = str.getProperty(properties::searching::TIMES_FOUND);
	if(uiValue)
		strFile << *uiValue;
	else
		strFile << "n/a";

	strFile << endl;
	// End title //////////////////

	////////////////////////////
	// Start atoms

	// Now write out the atom positions along with the spcies
  const ::std::string * species;
	for(size_t i = 0; i < str.getNumAtoms(); ++i)
	{
    const common::Atom & atom = str.getAtom(i);
    const ::arma::vec3 & pos = atom.getPosition();
    species = speciesDb.getSymbol(atom.getSpecies());

    if(species)
      strFile << *species << " ";
    else
      strFile << "DU ";
    strFile << ::std::setprecision(12) << pos(X) << " " << pos(Y) << " " << pos(Z) << ::std::endl;
	}

	// End atoms ///////////

  str.setProperty(
    properties::io::LAST_ABS_FILE_PATH,
    io::ResourceLocator(io::absolute(filepath)));

 if(strFile.is_open())
    strFile.close();
}

std::vector<std::string> XyzReaderWriter::getSupportedFileExtensions() const
{
	std::vector<std::string> ext;
	ext.push_back("xyz");
	return ext;
}

bool XyzReaderWriter::multiStructureSupport() const
{
  return false;
}

}
}
