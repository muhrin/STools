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

#include "build_cell/IFragmentGenerator.h"
#include "build_cell/IStructureGenerator.h"
#include "build_cell/IUnitCellGenerator.h"
#include "build_cell/BuildCellFwd.h"
#include "build_cell/PointGroups.h"

// FORWARD DECLARES //////////////////////////

namespace sstbx {
namespace build_cell {
namespace detail {

class StructureBuilderCore
{
public:
  
  template <class T>
  void addGenerator(SSLIB_UNIQUE_PTR(T) generator);

protected:
  typedef ::boost::ptr_vector<IFragmentGenerator> Generators;

  Generators myGenerators;
};

template <class T>
void StructureBuilderCore::addGenerator(SSLIB_UNIQUE_PTR(T) generator)
{
  myGenerators.push_back(generator.release());
}

} // namespace detail

class StructureBuilder : public detail::StructureBuilderCore, public IStructureGenerator
{
public:
  StructureBuilder();
  StructureBuilder(const StructureBuilder & toCopy);
  
  virtual GenerationOutcome generateStructure(
    common::StructurePtr & structureOut,
    const common::AtomSpeciesDatabase & speciesDb
  );

  void setUnitCellGenerator(IUnitCellGeneratorPtr unitCellGenerator);
  const IUnitCellGenerator * getUnitCellGenerator() const;

  void setPointGroup(const PointGroup & pointGroup);
  const PointGroup & getPointGroup() const;

  void setCluster(const bool isCluster);
  bool isCluster() const;

private:

  bool chooseSymmetry(StructureBuild & build) const;
  GenerationOutcome generateSymmetry(StructureBuild & build) const;

  PointGroup myPointGroup;
  unsigned int myNumSymOps;
  bool myIsCluster;

  IUnitCellGeneratorPtr myUnitCellGenerator;
};

}
}

#endif /* STRUCTURE_BUILDER_H */
