/*
 * ICrystalStructureGenerator.h
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef I_CRYSTAL_STRUCTURE_GENERATOR_H
#define I_CRYSTAL_STRUCTURE_GENERATOR_H

// FORWARD DECLARES //////////////////////////
namespace sstbx
{
namespace build_cell
{
template <typename FloatType>
class RandomCellDescription;
class StructureDescription;
}
namespace common
{
class Structure;
}
}

namespace sstbx { namespace build_cell {

class ICrystalStructureGenerator
{
public:

	/**
	 * Generate a new structure.
	 *
	 */
  virtual ::sstbx::common::Structure * const generateStructure(
    const StructureDescription & strDesc,
    const RandomCellDescription<double> & cellDesc) const = 0;
};

}}

#endif /* I_CRYSTAL_STRUCTURE_GENERATOR_H */
