/*
 * Types.h
 *
 *  Created on: Aug 30, 2012
 *      Author: Martin Uhrin
 */

#ifndef BUILD_CELL__TYPES_H
#define BUILD_CELL__TYPES_H

// INCLUDES ////////////
#include "SSLib.h"

namespace sstbx {
namespace build_cell {

// FORWARD DECLARES ////////////
class AtomsDescription;
class AtomsGenerator;
class IFragmentGenerator;
class IStructureGenerator;
class IUnitCellGenerator;
class RandomUnitCellGenerator;
class StructureBuilder;


typedef UniquePtr<AtomsDescription>::Type AtomsDescriptionPtr;
typedef UniquePtr<AtomsGenerator>::Type AtomsGeneratorPtr;
typedef UniquePtr<IFragmentGenerator>::Type IFragmentGeneratorPtr;
typedef UniquePtr<IUnitCellGenerator>::Type UnitCellGeneratorPtr;
typedef UniquePtr<IStructureGenerator>::Type IStructureGeneratorPtr;
typedef UniquePtr<RandomUnitCellGenerator>::Type RandomUnitCellPtr;
typedef UniquePtr<StructureBuilder>::Type StructureBuilderPtr;


}
}

#endif /* BUILD_CELL__TYPES_H */
