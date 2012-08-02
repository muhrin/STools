/*
 * ResReaderWriter.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "io/ResReaderWriter.h"

#include <iomanip>
#include <set>
#include <vector>

#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/tokenizer.hpp>

#include <armadillo>

#include "io/AdditionalData.h"
#include "common/AbstractFmidCell.h"
#include "common/Atom.h"
#include "common/AtomSpeciesDatabase.h"
#include "common/AtomSpeciesId.h"
#include "common/AtomSpeciesInfo.h"
#include "common/Structure.h"
#include "common/Types.h"
#include "utility/BoostFilesystem.h"

// DEFINES /////////////////////////////////


// NAMESPACES ////////////////////////////////


namespace sstbx {
namespace io {

namespace common = ::sstbx::common;

void ResReaderWriter::writeStructure(
	const sstbx::common::Structure &str,
	const::boost::filesystem::path & filepath,
	const AdditionalData *const data) const
{
	writeStructure(str, filepath, ::sstbx::common::AtomSpeciesDatabase::inst(), data);
}

void ResReaderWriter::writeStructure(
	const ::sstbx::common::Structure & str,
	const ::boost::filesystem::path & filepath,
	const ::sstbx::common::AtomSpeciesDatabase & speciesDb,
	const AdditionalData * const data) const
{
	using namespace ::boost::filesystem;
	using ::sstbx::common::AtomSpeciesId;
	using ::std::endl;
	using ::boost::filesystem::ofstream;

	if(!filepath.has_filename())
		throw "Cannot write out structure without filepath";

	const path dir = filepath.parent_path();
	if(!dir.empty() && !exists(dir))
	{
		create_directories(dir);
	}

	ofstream strFile;
	strFile.open(filepath);

	//////////////////////////
	// Start title
	strFile << "TITL ";
	
	if(data && data->name)
		strFile << *data->name;
	else
		strFile << filepath.stem();
	
	// Presssure
	strFile << " ";
	if(data && data->pressure)
		strFile << *data->pressure;
	else
		strFile << "n/a";

	// Volume
	strFile << " " << str.getUnitCell()->getVolume();

	// Enthalpy
	strFile << " ";
	if(data && data->enthalpy)
		strFile << *data->enthalpy;
	else
		strFile << "n/a";

	// Space group
	strFile << " 0 0 (";
	if(data && data->spaceGroup)
		strFile << *data->spaceGroup;
	else
		strFile << "n/a";

	// Times found
	strFile << ") n - ";
	if(data && data->timesFound)
		strFile << *data->timesFound;
	else
		strFile << "n/a";

	strFile << endl;
	// End title //////////////////
	
	///////////////////////////////////
	// Start lattice
	const double (&latticeParams)[6] = str.getUnitCell()->getLatticeParams();

	// Do cell parameters
	strFile << "CELL 1.0";
	for(size_t i = 0; i < 6; ++i)
	{
		strFile << " " << latticeParams[i];
	}
	strFile << endl;

	strFile << "LATT -1" << endl;

	// End lattice

	////////////////////////////
	// Start atoms

	// Get the species and positions of all atoms
	using std::vector;
	using std::set;

	arma::Mat<double> positions;
	str.getAtomPositionsDescendent(positions);

  vector<AtomSpeciesId::Value> species;
	str.getAtomSpeciesDescendent(species);

  set<AtomSpeciesId::Value> uniqueSpecies(species.begin(), species.end());

	// Output atom species
  std::map<AtomSpeciesId::Value, std::string> speciesSymbols;
  std::map<AtomSpeciesId::Value, unsigned int> speciesOrder;
	strFile << "SFAC";
  size_t idx = 1;
  BOOST_FOREACH(const AtomSpeciesId::Value id, uniqueSpecies)
	{
		const ::std::string * const name = speciesDb.getSymbol(id);
		speciesSymbols[id] = name ? *name : "?";
    speciesOrder[id]   = idx;
    ++idx;
		strFile << " " << speciesSymbols[id];
	}

	// Now write out the atom positions along with the spcies
	const sstbx::common::AbstractFmidCell * cell = str.getUnitCell();
	sstbx::common::Atom::Vec3 fracPos;
	for(size_t i = 0; i < positions.n_cols; ++i)
	{
    const AtomSpeciesId::Value id = species[i];
		fracPos = cell->fractionalise(positions.col(i));
		strFile << endl << ::std::setprecision(32) << speciesSymbols[id] << " " << speciesOrder[id] << " " <<
			fracPos[0] << " " << fracPos[1] << " " << fracPos[2] << " 1.0";
	}

	// End atoms ///////////

	strFile << endl << "END" << endl;


 if(strFile.is_open())
    strFile.close();
}

void ResReaderWriter::readStructure(
	sstbx::common::Structure &        str,
	const boost::filesystem::path &   filepath,
	AdditionalData * const            data) const
{
  readStructure(str, filepath, ::sstbx::common::AtomSpeciesDatabase::inst(), data);
}

void ResReaderWriter::readStructure(
	sstbx::common::Structure &          str,
	const boost::filesystem::path &     filepath,
	const sstbx::common::AtomSpeciesDatabase & speciesDb,
	AdditionalData * const              data) const
{
  namespace utility = ::sstbx::utility;
  using sstbx::common::AbstractFmidCell;
  using sstbx::common::Atom;
	using sstbx::common::AtomSpeciesId;
	using std::endl;
  using std::getline;
  using boost::bad_lexical_cast;
  using boost::lexical_cast;
	using boost::filesystem::ifstream;

	if(!filepath.has_filename())
		throw "Cannot write out structure without filepath";

  if(!exists(filepath))
    return;

	ifstream strFile;
	strFile.open(filepath);

  // Set up our tokenizer to split around space and tab
	typedef boost::tokenizer<boost::char_separator<char> > Tok;
	const boost::char_separator<char> sep(" \t");

  if(strFile.is_open())
  {
    std::string line;

    // We're expecting the TITL line
    bool foundTitle = false;
    for(
      getline(strFile, line);
      !foundTitle && strFile.good();
      getline(strFile, line))
    {
      Tok toker(line, sep);
      Tok::iterator tokIt = toker.begin();
      if(*tokIt == "TITL")
      {
        foundTitle = true;
        if(++tokIt != toker.end())
          str.setName(*tokIt);
        else
          str.setName(utility::stemString(filepath));

        // Does the user want additional data?
        if(data)
        {
          // First set the name
          data->name.reset(str.getName());

          bool hasMore = true;
          // Parse the rest of the tokens
          // Pressure
          if(hasMore && ++tokIt != toker.end())
          {
            try
            {
              data->pressure.reset(lexical_cast<double>(*tokIt));
            }
            catch(const bad_lexical_cast &)
            {}
          }
          else
            hasMore = false;

          // Volume
          if(hasMore && ++tokIt == toker.end())
            hasMore = false;

          // Free energy
          if(hasMore && ++tokIt != toker.end())
          {
            try
            {
              data->enthalpy.reset(lexical_cast<double>(*tokIt));
            }
            catch(const bad_lexical_cast &)
            {}
          }
          else 
            hasMore = false;

          if(hasMore && ++tokIt == toker.end())
            hasMore = false;
          if(hasMore && ++tokIt == toker.end())
            hasMore = false;

          // Space group
          if(hasMore && ++tokIt != toker.end())
            data->spaceGroup.reset(*tokIt);
          else
            hasMore = false;

          if(hasMore && ++tokIt == toker.end())
            hasMore = false;
          if(hasMore && ++tokIt == toker.end())
            hasMore = false;

          // Times found
          if(hasMore && ++tokIt != toker.end())
          {
            try
            {
              data->timesFound.reset(lexical_cast<size_t>(*tokIt));
            }
            catch(const bad_lexical_cast &)
            {}
          }
          else
            hasMore = false;


        } // end if(data)
      } // end if(*tokIt == "TITL")
    } // end for

    // We're expecting the CELL line
    bool foundCell = false;
    for(; // The previous for statement will have called one last getline
      !foundCell && strFile.good();
      getline(strFile, line))
    {
      Tok toker(line, sep);
      Tok::iterator tokIt = toker.begin();
      if(*tokIt == "CELL")
      {
        foundCell = true;

        // Move the token on
        bool hasMore = ++tokIt != toker.end();

        if(hasMore)
        {
          // Set up the cell parameters
          double params[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

          unsigned int i = 0;
          bool foundParams = true;
          for(++tokIt; i < 6 && tokIt != toker.end();
            ++i, ++tokIt)
          {
            try
            {
              params[i] = lexical_cast<double>(*tokIt);
            }
            catch(const bad_lexical_cast &)
            {
              foundParams = false;
              break;
            }
          }
          // Check if we found all six values
          foundParams = foundParams && i == 6;

          str.setUnitCell(common::UnitCellPtr(new AbstractFmidCell(params)));
        } // end if(hasMore)
      } // end if(*tokIt == "CELL")
    } // while !foundCell


    // Look for SFAC line
    bool foundSfac = false;
    for(; // The previous for statement will have called one last getline
      !foundSfac && strFile.good();
      getline(strFile, line))
    {
      Tok toker(line, sep);
      Tok::iterator tokIt = toker.begin();
      if(*tokIt == "SFAC")
      {
        foundSfac = true;

        // Skip over the first line, it just outlines overall species
        getline(strFile, line);

        // Now loop over all atoms
        bool foundEnd = false;
        for(; // The previous for statement will have called one last getline
          !foundEnd && strFile.good();
          getline(strFile, line))
        {
          bool atomFound = true;      

          Tok atomToker(line, sep);
          Tok::iterator atomTokIt = atomToker.begin();

          if(atomTokIt == atomToker.end() || *atomTokIt == "END")
          {
            foundEnd = true;
            break;
          }

          // Try finding the species id
          const AtomSpeciesId::Value id = speciesDb.getIdFromSymbol(*atomTokIt);

          if(id != sstbx::common::AtomSpeciesId::DUMMY)
          {
            bool hasMore = true;
            Atom * atom = new Atom(id);

            // Skip over first value
            hasMore = (++atomTokIt != atomToker.end());

            if(hasMore)
            {
              // Try to get the coordinates
              bool readCoordinates = true;
              arma::vec3 pos;
              unsigned int coord = 0;
              for(++atomTokIt;
                coord < 3 && atomTokIt != atomToker.end();
                ++coord, ++atomTokIt)
              {
                try
                {
                  pos(coord) = lexical_cast<double>(*atomTokIt);
                }
                catch(const bad_lexical_cast &)
                {
                  readCoordinates = false;
                  break;
                }
              }
              readCoordinates = readCoordinates && coord == 3;

              if(readCoordinates)
              {
                const AbstractFmidCell * const cell = str.getUnitCell();
                // Try to orthoginalise the position
                if(cell)
                {
                  atom->setPosition(cell->orthogonaliseInplace(pos));
                }
                str.insertAtom(atom);
              }
              else
              {
                // Didn't succeed
                delete atom;
                atom = NULL;
              } // end if(readCoordinates)
            } // end if(hasMore)
          }

        } // for all atoms

      } // end if(*tokIt == "SFAC")
    } // while !foundSfac
  
    strFile.close();
  } // end if(strFile.is_open())
}

std::vector<std::string> ResReaderWriter::getSupportedFileExtensions() const
{
	std::vector<std::string> ext;
	ext.push_back("res");
	return ext;
}

}}
