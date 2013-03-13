/*
 * RandomStructure.h
 *
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef RANDOM_STRUCTURE_H
#define RANDOM_STRUCTURE_H

// INCLUDES /////////////////////////////////////////////
#include "StructurePipe.h"

#include <boost/noncopyable.hpp>
#include <boost/optional.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/variant.hpp>

// From SSTbx
#include <build_cell/BuildCellFwd.h>

#include <pipelib/pipelib.h>

#include "SpTypes.h"

// FORWARD DECLARATIONS ////////////////////////////////////

namespace spipe {
namespace blocks {

class RandomStructure : public virtual SpStartBlock, public virtual SpPipeBlock,
  ::boost::noncopyable
{
public:

  typedef ::sstbx::build_cell::IStructureGeneratorPtr IStructureGeneratorPtr;

	RandomStructure(
    const int numToGenerate,
    IStructureGeneratorPtr structureGenerator = IStructureGeneratorPtr()
  );

  RandomStructure(
    const float atomsMultiplierGenerate,
    IStructureGeneratorPtr structureGenerator = IStructureGeneratorPtr()
  );


  // From StartBlock ///
	virtual void start();
  // End from StartBlock

  // From PipeBlock //
	virtual void in(::spipe::common::StructureData & data);
  // End from PipeBlock

private:
  typedef ::boost::scoped_ptr< ::sstbx::build_cell::IStructureGenerator> StructureGeneratorPtr;

  const ::sstbx::build_cell::IStructureGenerator * getStructureGenerator() const;

	const IStructureGeneratorPtr myStructureGenerator;
  const bool myFixedNumGenerate;
  const int myNumToGenerate;
  const float myAtomsMultiplierGenerate;
};


}
}

#endif /* RANDOM_STRUCTURE_H */
