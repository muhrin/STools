/*
 * AtomExtruderTest.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "sslibtest.h"

#include <iterator>

#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>

#include <spl/common/AtomSpeciesId.h>
#include <spl/common/Structure.h>
#include <spl/common/Types.h>
#include <spl/common/UnitCell.h>
#include <spl/io/CellReaderWriter.h>
#include <spl/io/ResourceLocator.h>
#include <spl/io/ResReaderWriter.h>
#include <spl/io/SplReaderWriter.h>
#include <spl/io/StructureReadWriteManager.h>
#include <spl/utility/StableComparison.h>

namespace fs = ::boost::filesystem;
namespace ssio = ::spl::io;
namespace ssc = ::spl::common;
namespace compare = ::spl::utility::stable;

void checkSimilar(const ssc::Structure & str1, const ssc::Structure & str2);

BOOST_AUTO_TEST_CASE(ReaderWriterTest)
{
  // SETTINGS ///////
  const size_t NUM_READER_WRITERS = 3;
  const size_t NUM_ATOMS = 32;
  const ::std::string SAVE_PATH = "rwTest.out";

  // Set up all the readers/writers we want to test
  ssio::StructureReadWriteManager rwMan;
  rwMan.insert(::spl::makeUniquePtr(new ssio::ResReaderWriter()));
  rwMan.insert(::spl::makeUniquePtr(new ssio::SplReaderWriter()));
  rwMan.insert(::spl::makeUniquePtr(new ssio::CellReaderWriter()));

  // Set up the structure to write
  ssc::Structure structure;
  ::arma::vec3 pos;
  ssc::AtomSpeciesId::Value species = "Na";
  for(size_t i = 0; i < NUM_ATOMS; ++i)
  {
    pos.randu();
    structure.newAtom(species).setPosition(pos);
  }
  
  ssio::ResourceLocator saveTo;
  saveTo.setId("rwTest");
  ssc::StructurePtr loadedStructure;
  for(ssio::StructureReadWriteManager::WritersIterator it = rwMan.beginWriters(),
    end = rwMan.endWriters(); it != end; ++it)
  {
    saveTo.setPath(fs::path(SAVE_PATH + "." + it->first));
    it->second->writeStructure(structure, saveTo);

    loadedStructure = rwMan.readStructure(saveTo);
    BOOST_CHECK(loadedStructure.get());

    if(loadedStructure.get())
      checkSimilar(structure, *loadedStructure);
  }

}

void checkSimilar(const ssc::Structure & str1, const ssc::Structure & str2)
{
  BOOST_REQUIRE(str1.getNumAtoms() == str2.getNumAtoms());

  ::std::vector<ssc::AtomSpeciesId::Value> species1, species2;
  str1.getAtomSpecies(::std::back_inserter(species1));
  str2.getAtomSpecies(::std::back_inserter(species2));
  BOOST_REQUIRE(species1.size() == species2.size());

  // Put all the species in structure 1 into a set and one by one insert those from
  // structure 2 and make sure no new ones are inserted
  ::std::set<ssc::AtomSpeciesId::Value> speciesSet(species1.begin(), species1.end());
  BOOST_FOREACH(const ssc::AtomSpeciesId::Value & spec, species2)
  {
    BOOST_REQUIRE(speciesSet.insert(spec).second == false);
    // Check the numbers of species match
    BOOST_REQUIRE(str1.getNumAtomsOfSpecies(spec) == str2.getNumAtomsOfSpecies(spec));
  }


  // Check the unit cell (if any)
  //const ssc::UnitCell * const uc1 = str1.getUnitCell();
  //const ssc::UnitCell * const uc2 = str2.getUnitCell();

  //BOOST_REQUIRE(!((uc1 == NULL) ^ (uc2 == NULL))); // Either have to be both NULL or both !NULL
  //if(uc1 && uc2)
  //{
  //  BOOST_REQUIRE(compare::eq(uc1->getVolume(), uc2->getVolume()));

  //  // TODO: More unit cell checks

  //}
}
