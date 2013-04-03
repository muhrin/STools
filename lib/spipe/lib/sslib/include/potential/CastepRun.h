/*
 * CastepRun.h
 *
 *
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef CASTEP_RUN_H
#define CASTEP_RUN_H

// INCLUDES /////////////////////////////////////////////

#include <string>

#include <boost/filesystem/fstream.hpp>
#include <boost/filesystem/path.hpp>

#include "potential/IGeomOptimiser.h" // TODO: REMOVE

// DEFINES //////////////////////////////////////////////

namespace sstbx {
// FORWARD DECLARATIONS ////////////////////////////////////
namespace common {
class AtomSpeciesDatabase;
class Structure;
}
namespace io {
class CellReaderWriter;
class CastepReader;
}
namespace potential {


struct CastepRunResult
{
  enum Value
  {
    SUCCESS,
    INPUT_NOT_FOUND,
    OUTPUT_NOT_FOUND,
    FAILED_TO_RUN,
    ERROR_READING_FILE,
    FAILED_TO_READ_STRUCTURE
  };
};

class CastepRun
{
public:
  CastepRun(
    const ::std::string & seed,
    const io::CellReaderWriter & myCellReaderWriter,
    const io::CastepReader & myCastepReader
  );
  ~CastepRun();

  const ::boost::filesystem::path & getParamFile() const;
  const ::boost::filesystem::path & getCellFile() const;
  const ::boost::filesystem::path & getCastepFile() const;

  CastepRunResult::Value openCellFile(::boost::filesystem::ofstream * * ofstream);
  CastepRunResult::Value openCastepFile(::boost::filesystem::ifstream * * ifstream = NULL);

  void closeAllStreams();

  CastepRunResult::Value runCastep(const ::boost::filesystem::path & castepExe);

  CastepRunResult::Value updateStructureFromOutput(
    common::Structure & structure,
    const common::AtomSpeciesDatabase & speciesDb
  );

  CastepRunResult::Value deleteAllOutput();
  CastepRunResult::Value deleteAllFiles();

private:
  const ::boost::filesystem::path myCellFile;
  const ::boost::filesystem::path myParamFile;
  const ::boost::filesystem::path myCastepFile;

  ::boost::filesystem::ofstream myCellFileStream;
  ::boost::filesystem::ifstream myCastepFileStream;

  const io::CellReaderWriter & myCellReaderWriter;
  const io::CastepReader & myCastepReader;
};

class CastepGeomOptRun
{
public:
  CastepGeomOptRun(
    const ::std::string & originalSeed,
    const ::std::string & newSeed,
    const bool keepIntermediates,
    const io::CellReaderWriter & myCellReaderWriter,
    const io::CastepReader & myCastepReader
  );
  ~CastepGeomOptRun();

  OptimisationOutcome runFullRelax(
    common::Structure & structure,
    PotentialData & data,
    const ::boost::filesystem::path & castepExe,
    const common::AtomSpeciesDatabase & speciesDb,
    const int numConsistentRelaxations = 3
  );
  bool copyParamFile() const;

private:

  static const int MAX_RELAX_ATTEMPTS;

  OptimisationOutcome makeCellCopy(
    common::Structure & structure,
    const common::AtomSpeciesDatabase & speciesDb
  );
  OptimisationOutcome doRelaxation(
    common::Structure & structure,
    const common::AtomSpeciesDatabase & speciesDb,
    const ::boost::filesystem::path & castepExe
  );
  bool updateOptimisationInfo(
    common::Structure & structure,
    PotentialData & data,
    const common::AtomSpeciesDatabase & speciesDb
  );
  bool optimistaionSucceeded();

  CastepRun myCastepRun;
  const ::boost::filesystem::path myOrigCellFile;
  const ::boost::filesystem::path myOrigParamFile;
  const bool myKeepIntermediates;
  const io::CellReaderWriter & myCellReaderWriter;
  const io::CastepReader & myCastepReader;
};

}
}

#endif /* CASTEP_RUN_H */
