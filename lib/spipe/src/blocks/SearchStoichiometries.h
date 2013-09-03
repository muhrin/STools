/*
 * SearchStoichiometries.h
 *
 *
 *  Created on: May 4, 2012
 *      Author: Martin Uhrin
 */

#ifndef SEARCH_STOICHIOMETRIES_H
#define SEARCH_STOICHIOMETRIES_H

// INCLUDES /////////////////////////////////////////////
#include "StructurePipe.h"

#include <vector>

#include <boost/scoped_ptr.hpp>
#include <boost/shared_ptr.hpp>

#include <pipelib/pipelib.h>

#include <spl/build_cell/StructureBuilder.h>
#include <spl/build_cell/RandomUnitCellGenerator.h>
#include <spl/build_cell/BuildCellFwd.h>
#include <spl/common/AtomSpeciesId.h>
#include <spl/io/BoostFilesystem.h>
#include <spl/utility/MultiIdxRange.h>

// Local includes
#include "SpTypes.h"
#include "utility/DataTable.h"
#include "utility/DataTableSupport.h"

// FORWARD DECLARATIONS ////////////////////////////////////
namespace spl {
namespace build_cell {
class AddOnStructureBuilder;
}
namespace common {
class AtomSpeciesDatabase;
}
}

namespace spipe {
namespace common {
class DataTableWriter;
}

namespace blocks {

struct SpeciesParameter
{
  SpeciesParameter(const ::spl::common::AtomSpeciesId::Value _id,
      const size_t _maxNum) :
      id(_id), maxNum(_maxNum)
  {
  }

  ::spl::common::AtomSpeciesId::Value id;
  size_t maxNum;
};

class SearchStoichiometries : public StartBlock,
    public FinishedSink,
    ::boost::noncopyable
{
public:
  typedef ::spl::build_cell::StructureBuilderPtr StructureBuilderPtr;
  typedef ::std::vector< SpeciesParameter> SpeciesParameters;

  SearchStoichiometries(const ::spl::common::AtomSpeciesId::Value species1,
      const ::spl::common::AtomSpeciesId::Value species2, const size_t maxAtoms,
      BlockHandle & subpipe, StructureBuilderPtr structureBuilder =
          StructureBuilderPtr());

  SearchStoichiometries(const SpeciesParameters & speciesParameters,
      const size_t maxAtoms, const double atomsRadius, BlockHandle & subpipe,
      StructureBuilderPtr structureBuilder = StructureBuilderPtr());

  // From Block ////////
  virtual void
  pipelineInitialising();
  virtual void
  pipelineStarting();
  // End from Block ////

  // From StartBlock ///
  virtual void
  start();
  // End from StartBlock ///

  // From IDataSink /////////////////////////////
  virtual void
  finished(StructureDataUniquePtr data);
  // End from IDataSink /////////////////////////

private:
  typedef ::spipe::StructureDataType StructureDataTyp;
  typedef ::boost::scoped_ptr< ::spipe::utility::DataTableWriter> TableWriterPtr;

  // From Block ////////
  virtual void
  engineAttached(Engine::SetupType * const setup);
  virtual void
  engineDetached();
  // End from Block ////

  ::spl::utility::MultiIdxRange< unsigned int>
  getStoichRange();

  void
  releaseBufferedStructures(const utility::DataTable::Key & key);

  void
  updateTable(const utility::DataTable::Key & key,
      const ::spl::utility::MultiIdx< unsigned int> & currentIdx,
      const ::spl::common::AtomSpeciesDatabase & atomsDb);

  StructureBuilderPtr
  newStructureGenerator() const;

  BlockHandle mySubpipe;
  Engine * mySubpipeEngine;

  // Use this to write out our table data
  ::spipe::utility::DataTableSupport myTableSupport;
  const size_t myMaxAtoms;
  ::boost::filesystem::path myOutputPath;

  /** Buffer to store structure that have finished their path through the sub pipeline. */
  ::std::vector< StructureDataTyp *> myBuffer;

  SpeciesParameters mySpeciesParameters;
  StructureBuilderPtr myStructureGenerator;
};

}
}

#endif /* SEARCH_STOICHIOMETRIES_H */
