/*
 * StructureData.h
 *
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef STRUCTURE_DATA_H
#define STRUCTURE_DATA_H

// INCLUDES /////////////////////////////////////////////
#include "StructurePipe.h"


#include <boost/optional.hpp>

#include <armadillo>

#include <utility/BoostFilesystem.h>
#include <utility/HeterogeneousMap.h>

// FORWARD DECLARATIONS ////////////////////////////////////
namespace sstbx
{
namespace common
{
class Structure;
}
}

namespace spipe
{
namespace common
{

struct StructureObjectKeys
{
  // The absolute path to the last place that a structure was saved to disk
  static const ::sstbx::utility::Key< ::boost::filesystem::path>  LAST_ABS_SAVE_PATH;
  static const ::sstbx::utility::Key<unsigned int>                SPACEGROUP_NUMBER;
  static const ::sstbx::utility::Key< ::std::string>              SPACEGROUP_SYMBOL;


};

class StructureData
{
public:
	StructureData();
	~StructureData();

	sstbx::common::Structure * getStructure() const;
	void setStructure(sstbx::common::Structure * const structure);

  /**
  /* Get the path to where this structure was last saved relative to the output path
  /* of a given structure pipe.
  /**/
  ::boost::filesystem::path getRelativeSavePath(const ::spipe::SpPipelineTyp & pipeline) const;

	::boost::optional<double>				enthalpy;
	::boost::optional<std::string>		name;
	::boost::optional<double>				pressure;
	::boost::optional<std::string>		spaceGroup;
	::boost::optional<arma::Mat<double> > stressMtx;
	::boost::optional<size_t>				timesFound;

  ::sstbx::utility::HeterogeneousMap  objectsStore;

private:

	::sstbx::common::Structure * myStructure;
};


}}

#endif /* STRUCTURE_DATA_H */
