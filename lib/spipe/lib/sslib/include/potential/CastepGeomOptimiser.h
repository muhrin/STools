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

#include "common/AtomSpeciesDatabase.h" // TODO: Remove this dependency
#include "potential/CastepRun.h"
#include "potential/IControllableOptimiser.h"
#include "potential/IOptimisationController.h"
#include "io/CastepReader.h"
#include "io/CellReaderWriter.h"
#include "math/RunningStats.h"

// DEFINES //////////////////////////////////////////////

namespace sstbx {
// FORWARD DECLARATIONS ////////////////////////////////////
namespace potential {

struct CastepGeomOptimiseSettings
{
  CastepGeomOptimiseSettings();
  int numRoughSteps;
  int numConsistentRelaxations;
  bool keepIntermediateFiles;
};

class CastepGeomOptimiser : public IControllableOptimiser
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
  // From IControllableOptimiser //////////
  virtual IOptimisationController * getController();
  virtual void setController(IOptimisationController & controller);
  // End from IControllableOptimiser //////

  void applySettings(const Settings & settings);

private:
  Settings mySettings;
  const ::std::string myCastepSeed;
  const io::CellReaderWriter myCellReaderWriter;
  const io::CastepReader myCastepReader;
  const common::AtomSpeciesDatabase mySpeciesDb; // HACK: Keep a copy here for now
  const ::std::string myCastepExe;
  IOptimisationController * myController;
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
    const CastepGeomOptimiseSettings & settings,
    IOptimisationController * controller = NULL
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

  class GeomRelaxer
  {
  public:
    GeomRelaxer(
      CastepRun & castepRun,
      const ::std::string & castepExeAndArgs,
      const io::CastepReader & castepReader,
      IOptimisationController * controller
    );

    OptimisationOutcome doRelaxation(
      common::Structure & structure,
      OptimisationData & optimistaionData,
      const common::AtomSpeciesDatabase & speciesDb
    );
    OptimisationOutcome updateStructure(
      common::Structure & structure,
      OptimisationData & data,
      const common::AtomSpeciesDatabase & speciesDb
    );
  private:
    OptimisationOutcome trackProgress(
      common::Structure & structure,
      OptimisationData & optimistaionData,
      const common::AtomSpeciesDatabase & speciesDb
    );
    bool CastepGeomOptRun::GeomRelaxer::parseOptimisationInfo(
      common::Structure & structure,
      OptimisationData & data,
      const common::AtomSpeciesDatabase & speciesDb
    );
    bool waitForStepToFinish(
      common::Structure & structure,
      OptimisationData & data,
      const common::AtomSpeciesDatabase & speciesDb
    );

    CastepRun & myCastepRun;
    const ::std::string myCastepExeAndArgs;
    const io::CastepReader & myCastepReader;
    IOptimisationController * myController;
    int myOptimisationStep;
    math::RunningStats myStepTimingStats;
  };

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

  CastepRun myCastepRun;
  const CastepGeomOptimiseSettings mySettings;
  const ::boost::filesystem::path myOrigCellFile;
  const ::boost::filesystem::path myOrigParamFile;
  const io::CellReaderWriter & myCellReaderWriter;
  const io::CastepReader & myCastepReader;
  const OptimisationSettings & myOptimisationSettings;
  IOptimisationController * myController;
};

} // namespace detail
} // namespace potential
} // namespace sstbx

#endif /* TPSD_GEOM_OPTIMISER_H */
