/*
 * StructureBuilder.h
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef STRUCTURE_BUILDER_H
#define STRUCTURE_BUILDER_H

// INCLUDES /////////////////////////////////

#include "SSLib.h"

#include <string>
#include <utility>

#include <boost/ptr_container/ptr_vector.hpp>

#include "build_cell/IStructureGenerator.h"
#include "build_cell/Types.h"

// FORWARD DECLARES //////////////////////////

namespace sstbx {
namespace build_cell {

class StructureBuilder : public IStructureGenerator
{
public:
  
  virtual GenerationOutcome generateStructure(
    common::StructurePtr & structureOut,
    const common::AtomSpeciesDatabase & speciesDb
  ) const;

  void setUnitCellGenerator(UnitCellGeneratorPtr unitCellGenerator);
  const IUnitCellGenerator * getUnitCellGenerator() const;

  template <class T>
  void addGenerator(SSLIB_UNIQUE_PTR(T) generator);

private:

  typedef ::boost::ptr_vector<IFragmentGenerator> Generators;
  
  UnitCellGeneratorPtr myUnitCellGenerator;
  Generators myGenerators;
  
};

template <class T>
void StructureBuilder::addGenerator(SSLIB_UNIQUE_PTR(T) generator)
{
  myGenerators.push_back(generator.release());
}

}
}

#endif /* STRUCTURE_BUILDER_H */
