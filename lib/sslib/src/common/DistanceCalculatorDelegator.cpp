/*
 * DistanceCalculatorDelegator.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES ///////////////
#include "common/DistanceCalculatorDelegator.h"

#include "common/ClusterDistanceCalculator.h"
#include "common/OrthoCellDistanceCalculator.h"
#include "common/UnitCell.h"
#include "common/UniversalCrystalDistanceCalculator.h"

namespace sstbx {
namespace common {

DistanceCalculatorDelegator::DistanceCalculatorDelegator(const Structure & structure):
DistanceCalculator(structure),
myDelegate(new ClusterDistanceCalculator(structure)),
myDelegateType(CalculatorType::CLUSTER)
{
  // WARNING: Don't use structure here as it won't be initialised!!
}

void DistanceCalculatorDelegator::unitCellChanged()
{
  updateDelegate();
}

void DistanceCalculatorDelegator::updateDelegate()
{
  const UnitCell * const unitCell = myStructure.getUnitCell();

  if(unitCell == NULL)
  {
    if(myDelegateType != CalculatorType::CLUSTER)
    {
      myDelegate.reset(new ClusterDistanceCalculator(myStructure));
      myDelegateType = CalculatorType::CLUSTER;
    }
  }
  else
  {
    const UnitCell::LatticeSystem::Value latticeSystem = unitCell->getLatticeSystem();
    if(latticeSystem == UnitCell::LatticeSystem::TETRAGONAL ||
      latticeSystem == UnitCell::LatticeSystem::CUBIC ||
      latticeSystem == UnitCell::LatticeSystem::ORTHORHOMBIC)
    {
      if(myDelegateType != CalculatorType::ORTHO_CELL)
      {
        myDelegate.reset(new OrthoCellDistanceCalculator(myStructure));
        myDelegateType = CalculatorType::ORTHO_CELL;
      }
    }
    else
    {
      if(myDelegateType != CalculatorType::UNIVERSAL_CRYSTAL)
      {
        myDelegate.reset(new UniversalCrystalDistanceCalculator(myStructure));
        myDelegateType = CalculatorType::UNIVERSAL_CRYSTAL;
      }
    }

  }
}

} // namespace sstbx
} // namespace common
