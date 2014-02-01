/*
 * ParamGeomOptimise.h
 * Parameterisable Potential Geometry Optimisation
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef PARAM_GEOM_OPTIMISE_H
#define PARAM_GEOM_OPTIMISE_H

// INCLUDES /////////////////////////////////////////////
#include "StructurePipe.h"

#include <map>


#include <armadillo>

#include <pipelib/pipelib.h>

#include "SpTypes.h"
#include "blocks/GeomOptimise.h"
#include "utility/DataTable.h"
#include "utility/DataTableSupport.h"

// FORWARD DECLARATIONS ////////////////////////////////////

namespace spl {
namespace potential {
class IParameterisable;
class GeomOptimiser;
}
}

namespace spipe {
namespace blocks {

class ParamGeomOptimise : public GeomOptimise
{
public:
  ParamGeomOptimise(::spl::potential::IGeomOptimiserPtr optimiser);
  ParamGeomOptimise(::spl::potential::IGeomOptimiserPtr optimiser,
      const GeomOptimise::Settings & settings);
  ParamGeomOptimise(::spl::potential::IGeomOptimiserPtr optimiser,
      const spl::potential::OptimisationSettings & optimisationParams);
  ParamGeomOptimise(::spl::potential::IGeomOptimiserPtr optimiser,
      const spl::potential::OptimisationSettings & optimisationParams,
      const GeomOptimise::Settings & settings);

  // From Block /////////////////////////
  virtual void
  pipelineInitialising();
  virtual void
  pipelineStarting();
  // End from Block ////////////////////

  // From PipeBlock ///////////////////////////
  virtual void
  in(common::StructureData * const data);
  // End from PipeBlock ///////////////////////

private:
  typedef std::vector< double> PotentialParams;

  void
  init();

  void
  setPotentialParams(const PotentialParams & params);

  spl::potential::IParameterisable * myParamPotential;
  PotentialParams myCurrentParams;
};

}
}

#endif /* PARAM_GEOM_OPTIMISE_H */
