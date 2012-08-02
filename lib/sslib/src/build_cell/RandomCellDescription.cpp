/*
 * RandomCellDescription.cpp
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */


// INCLUDES //////////////////////////////////////
#include "build_cell/RandomCellDescription.h"

namespace sstbx {
namespace build_cell {

void RandomCellDescription::setLatticeParams(
	const double a, const double b, const double c,
	const double alpha, const double beta, const double gamma)
{
  myA.reset(a);
  myB.reset(b);
  myC.reset(b);

  myAlpha.reset(alpha);
  myBeta.reset(beta);
  myGamma.reset(gamma);
}

void RandomCellDescription::setLatticeParams(const double (&latticeParams)[6])
{
  myA.reset(latticeParams[0]);
	myB.reset(latticeParams[1]);
	myC.reset(latticeParams[2]);
	myAlpha.reset(latticeParams[3]);
	myBeta.reset(latticeParams[4]);
	myGamma.reset(latticeParams[5]);
}


}
}
