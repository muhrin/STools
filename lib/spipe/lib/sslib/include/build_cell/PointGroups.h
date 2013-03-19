/*
 * PointGroups.h
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef POINT_GROUPS_H
#define POINT_GROUPS_H

// INCLUDES ////////////
#include "build_cell/SymmetryGroup.h"
#include "common/Constants.h"

// DEFINITION ///////////////////////

namespace sstbx {
namespace build_cell {

class C3v : public SymmetryGroup
{
public:
  C3v()
  {
    mySymOps.resize(3);
    double angle;
    for(int i = 1; i < 3; ++i)
    {
      angle = static_cast<double>(i) * common::constants::TWO_PI / 3.0;
      mySymOps[i] 
        << cos(angle) << -sin(angle) << 0.0 << 0.0 << ::arma::endr
        << sin(angle) << cos(angle) << 0.0 << 0.0 << ::arma::endr
        << 0.0 << 0.0 << 1.0 << 0.0 << ::arma::endr
        << 0.0 << 0.0 << 0.0 << 0.0 << ::arma::endr;
    }
    generateMultiplicityEigenvectors();
  }
};

}
}


#endif /* POINT_GROUPS_H */
