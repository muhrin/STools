/*
 * CastepGeomOptimiser.h
 *
 *
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef CASTEP_GEOM_OPTIMISER_H
#define CASTEP_GEOM_OPTIMISER_H

// INCLUDES /////////////////////////////////////////////

#include <boost/filesystem/path.hpp>

#include "spl/common/AtomSpeciesDatabase.h" // TODO: Remove this dependency
#include "spl/potential/CastepRun.h"
#include "spl/potential/IGeomOptimiser.h"
#include "spl/io/CastepReader.h"
#include "spl/io/CellReaderWriter.h"

// DEFINES //////////////////////////////////////////////

namespace spl {
// FORWARD DECLARATIONS ////////////////////////////////////
namespace potential {

struct CastepGeomOptimiseSettings
{
  CastepGeomOptimiseSettings();
  int numRoughSteps;
  int numConsistentRelaxations;
  bool keepIntermediateFiles;
};

class CastepGeomOptimiser : public IGeomOptimiser
{
public:

  typedef CastepGeomOptimiseSettings Settings;

  static const ::std::string FINAL_ENTHALPY;

  CastepGeomOptimiser(
    const ::std::string & castepExe,
    const ::std::string & castepSeed
  );

  CastepGeomOptimiser(
    const ::std::string & castepExe,
    const ::std::string & castepSeed,
    const Settings & settings
  );

  // From IGeomOptimiser ////////
  // Don't have a potential
  virtual IPotential * getPotential() { return NULL; }
  virtual const IPotential * getPotential() const { return NULL; };

  virtual OptimisationOutcome optimise(
    common::Structure & structure,
    const OptimisationSettings & options
  ) const;
  virtual OptimisationOutcome optimise(
    common::Structure & structure,
    OptimisationData & data,
    const OptimisationSettings & options
  ) const;
  // End from IGeomOptimiser //////////////

  void applySettings(const Settings & settings);

private:
  Settings mySettings;
  const ::std::string myCastepSeed;
  const io::CellReaderWriter myCellReaderWriter;
  const io::CastepReader myCastepReader;
  const common::AtomSpeciesDatabase mySpeciesDb; // HACK: Keep a copy here for now
  const ::std::string myCastepExe;
};

namespace detail {
  
class CastepGeomOptRun
{
public:
  CastepGeomOptRun(
    const OptimisationSettings & optimisationSettings,
    const ::std::string & originalSeed,
    const ::std::string & newSeed,
    const io::CellReaderWriter & myCellReaderWriter,
    const io::CastepReader & myCastepReader,
    const CastepGeomOptimiseSettings & settings
  );
  ~CastepGeomOptRun();

  OptimisationOutcome runFullRelax(
    common::Structure & structure,
    OptimisationData & data,
    const ::std::string & castepExeAndArgs,
    const common::AtomSpeciesDatabase & speciesDb
  );
  OptimisationOutcome updateStructure(
    common::Structure & structure,
    OptimisationData & data,
    const common::AtomSpeciesDatabase & speciesDb
  );

private:
  static const int MAX_RELAX_ATTEMPTS;

  bool copyParamFile() const;
  OptimisationOutcome makeCellCopy(
    common::Structure & structure,
    const common::AtomSpeciesDatabase & speciesDb
  );
  OptimisationOutcome doPreRelaxation(
    common::Structure & structure,
    OptimisationData & optimisationData,
    const common::AtomSpeciesDatabase & speciesDb,
    const ::std::string & castepExeAndArgs
  );
  OptimisationOutcome doRelaxation(
    common::Structure & structure,
    OptimisationData & optimistaionData,
    const common::AtomSpeciesDatabase & speciesDb,
    const ::std::string & castepExeAndArgs
  );
  bool optimisationSucceeded();
  bool parseOptimisationInfo(
    common::Structure & structure,
    OptimisationData & data,
    const common::AtomSpeciesDatabase & speciesDb
  );

  CastepRun myCastepRun;
  const CastepGeomOptimiseSettings mySettings;
  const ::boost::filesystem::path myOrigCellFile;
  const ::boost::filesystem::path myOrigParamFile;
  const io::CellReaderWriter & myCellReaderWriter;
  const io::CastepReader & myCastepReader;
  const OptimisationSettings & myOptimisationSettings;
};

} // namespace detail
} // namespace potential
} // namespace spl

#endif /* TPSD_GEOM_OPTIMISER_H */
