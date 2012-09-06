/*
 * SharedData.h
 *
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef SHARED_DATA_H
#define SHARED_DATA_H

// INCLUDES /////////////////////////////////////////////
#include "StructurePipe.h"

#include <boost/optional.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/filesystem.hpp>

#include <armadillo>

#include <pipelib/event/IPipeListener.h>

#include <common/AtomSpeciesDatabase.h>
#include <utility/BoostFilesystem.h>
#include <utility/HeterogeneousMap.h>

// Local includes
#include "utility/DataTable.h"

// FORWARD DECLARATIONS ////////////////////////////////////

namespace pipelib {
namespace event {
template <class Listener>
class PipeStateChanged;
}
}

namespace sstbx {
namespace build_cell {
class StructureDescription;
}
}

namespace spipe
{
namespace common
{

struct GlobalKeys
{
  // The current parameterised potential parameters
  static const ::sstbx::utility::Key< ::arma::vec>  POTENTIAL_PARAMS;


};

class SharedData : public ::pipelib::event::IPipeListener< ::spipe::SpPipelineTyp>
{
public:

  static const char DIR_SUBSTRING_DELIMITER[];

  SharedData();
  ~SharedData();

  /**
  /* Set the pipeline that this shared data belong to.  This should be done immediately
  /* after constructing the pipeline.
  /* 
  /* Precondition: the pipeline must not have been set previously.
  /**/
  void setPipe(::spipe::SpPipelineTyp & pipe);

  bool appendToOutputDirName(const ::std::string & toAppend);

  /**
  /* Get the output path for the pipeline that owns this shared data relative to
  /* the working directory where the code was executed.
  /**/
  ::boost::filesystem::path getOutputPath() const;

  /**
  /* Get the output path for the pipeline that owns this shared data relative to
  /* the parent pipeline (or global data output path if there is not parent).
  /**/
  const ::boost::filesystem::path & getPipeRelativeOutputPath() const;

  /**
  /* Get a stem filename for output being made by blocks within the pipeline that
  /* owns this shared data.
  /* Using this as the stem output filename allows output from a particular run
  /* through the pipeline to be easily identified.
  /*/
  const ::boost::filesystem::path & getOutputFileStem() const;

  ::sstbx::build_cell::StructureDescription * getStructureDescription();

  ::sstbx::common::AtomSpeciesDatabase & getSpeciesDatabase();

  // From IPipeListener ////////////////////////
  virtual void notify(const ::pipelib::event::PipeStateChanged< ::spipe::SpPipelineTyp> & evt);
  // End from IPipeListener ///////////////////

	/** Potential sweep starting values */
	::boost::optional< ::arma::vec>			potSweepFrom;
	/** Potential sweep deltas */
	::boost::optional< ::arma::vec>			potSweepStep;
	/** Potential sweep number of steps to make */
	::boost::optional< ::arma::Col< unsigned int > >	potSweepNSteps;

  ::boost::shared_ptr< sstbx::build_cell::StructureDescription>            structureDescription;

  ::std::string                       outputFilename;

  ::sstbx::utility::HeterogeneousMap  objectsStore;

  ::spipe::utility::DataTable          dataTable;

private:

  void reset();

  void buildOutputPathRecursive(::boost::filesystem::path & path, const ::spipe::SpPipelineTyp & pipe) const;

  ::spipe::SpPipelineTyp *            myPipe;

  ::sstbx::common::AtomSpeciesDatabase  mySpeciesDatabase;
  ::boost::filesystem::path             myOutputDir;
  ::boost::filesystem::path             myOutputFileStem;

};


}
}

#endif /* SHARED_DATA_H */
