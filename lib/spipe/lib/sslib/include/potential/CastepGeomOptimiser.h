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
    PotentialData & data,
    const OptimisationSettings & options
  ) const;
  // End from IGeomOptimiser //////////////
private:
  static const ::std::string FINAL_ENTHALPY;

  bool updateStructure(
    common::Structure & structure,
    PotentialData & data,
    const ::boost::filesystem::path & castepFile,
    const common::AtomSpeciesDatabase & speciesDb
  ) const;
  bool parseOptimisationData(
    PotentialData & data,
    ::std::istream & inputStream
  ) const;

  const ::std::string myCastepSeed;
  const io::CellReaderWriter myCellReaderWriter;
  const io::CastepReader myCastepReader;
  const common::AtomSpeciesDatabase mySpeciesDb; // HACK: Keep a copy here for now
  const bool myKeepIntermediates;
  const ::boost::filesystem::path myCastepExe;
};

}
}

#endif /* TPSD_GEOM_OPTIMISER_H */
