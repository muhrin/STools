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
#include "potential/IGeomOptimiser.h"
#include "io/CastepReader.h"
#include "io/CellReaderWriter.h"

// DEFINES //////////////////////////////////////////////

namespace sstbx {
// FORWARD DECLARATIONS ////////////////////////////////////
namespace potential {

class CastepGeomOptimiser : public IGeomOptimiser
{
public:
  static const ::std::string FINAL_ENTHALPY;

  CastepGeomOptimiser(
    const ::boost::filesystem::path & castepExe,
    const ::std::string & castepSeed,
    const bool keepIntermediates
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

private:
  const ::std::string myCastepSeed;
  const io::CellReaderWriter myCellReaderWriter;
  const io::CastepReader myCastepReader;
  const common::AtomSpeciesDatabase mySpeciesDb; // HACK: Keep a copy here for now
  const bool myKeepIntermediates;
  const ::boost::filesystem::path myCastepExe;
};

namespace detail {
  
class CastepGeomOptRun
{
public:
  CastepGeomOptRun(
    const OptimisationSettings & optimisationSettings,
    const ::std::string & originalSeed,
    const ::std::string & newSeed,
    const bool keepIntermediates,
    const io::CellReaderWriter & myCellReaderWriter,
    const io::CastepReader & myCastepReader
  );
  ~CastepGeomOptRun();

  OptimisationOutcome runFullRelax(
    common::Structure & structure,
    OptimisationData & data,
    const ::boost::filesystem::path & castepExe,
    const common::AtomSpeciesDatabase & speciesDb,
    const int numConsistentRelaxations = 3
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
    const ::boost::filesystem::path & castepExe
  );
  OptimisationOutcome doRelaxation(
    common::Structure & structure,
    OptimisationData & optimistaionData,
    const common::AtomSpeciesDatabase & speciesDb,
    const ::boost::filesystem::path & castepExe
  );
  bool optimisationSucceeded();
  bool parseOptimisationInfo(
    common::Structure & structure,
    OptimisationData & data,
    const common::AtomSpeciesDatabase & speciesDb
  );

  CastepRun myCastepRun;
  const ::boost::filesystem::path myOrigCellFile;
  const ::boost::filesystem::path myOrigParamFile;
  const bool myKeepIntermediates;
  const io::CellReaderWriter & myCellReaderWriter;
  const io::CastepReader & myCastepReader;
  const OptimisationSettings & myOptimisationSettings;
};

} // namespace detail
} // namespace potential
} // namespace sstbx

#endif /* TPSD_GEOM_OPTIMISER_H */
