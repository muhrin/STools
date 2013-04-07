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

#include <map>
#include <string>

#include <boost/filesystem/fstream.hpp>
#include <boost/filesystem/path.hpp>

#include <armadillo>

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
  typedef ::std::map< ::std::string, ::std::string> ParamsMap;

  CastepRun(
    const ::std::string & seed,
    const io::CellReaderWriter & myCellReaderWriter,
    const io::CastepReader & myCastepReader
  );
  ~CastepRun();

  const ::boost::filesystem::path & getParamFile() const;
  const ::boost::filesystem::path & getCellFile() const;
  const ::boost::filesystem::path & getCastepFile() const;
  const ::boost::filesystem::path & getCellOutFile() const;

  CastepRunResult::Value openCellFile(
    ::boost::filesystem::ofstream * * ofstream = NULL,
    const bool atBeginning = true
  );
  void deleteCellFile();
  CastepRunResult::Value openCastepFile(::boost::filesystem::ifstream * * ifstream = NULL);
  CastepRunResult::Value openCellOutFile(::boost::filesystem::ifstream * * ifstream = NULL);

  CastepRunResult::Value insertParams(ParamsMap params) const;
  CastepRunResult::Value writePressure(const ::arma::mat33 & pressureTensor);

  void closeAllStreams();

  CastepRunResult::Value runCastep(const ::boost::filesystem::path & castepExe);

  CastepRunResult::Value updateStructureFromOutput(
    common::Structure & structure,
    const common::AtomSpeciesDatabase & speciesDb
  );

  CastepRunResult::Value deleteAllOutput();
  CastepRunResult::Value deleteAllFiles();

  bool finishedSuccessfully();

private:
  const ::boost::filesystem::path myCellFile;
  const ::boost::filesystem::path myParamFile;
  const ::boost::filesystem::path myCastepFile;
  const ::boost::filesystem::path myCellOutFile;

  ::boost::filesystem::ofstream myCellFileStream;
  ::boost::filesystem::fstream myParamFileStream;
  ::boost::filesystem::ifstream myCastepFileStream;
  ::boost::filesystem::ifstream myCellOutFileStream;

  const io::CellReaderWriter & myCellReaderWriter;
  const io::CastepReader & myCastepReader;
};

}
}

#endif /* CASTEP_RUN_H */
