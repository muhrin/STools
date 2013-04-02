/*
 * Matrix.h
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef MATRIX_H
#define MATRIX_H

// INCLUDES ////////////
#include "SSLib.h"

#include <armadillo>

// DEFINITION ///////////////////////

namespace sstbx {
namespace math {

void normalise(::arma::vec & vec);
::arma::vec normaliseCopy(const ::arma::vec & vec);

bool equals(const ::arma::mat & a, const ::arma::mat & b);
bool equals(const ::arma::mat & a, const ::arma::mat & b, const double tolerance);

bool isZero(const ::arma::mat & mat);
bool isZero(const ::arma::mat & mat, const double tolerance);

bool isSpannedBy(const ::arma::mat & superspace, const ::arma::mat & subspace);

bool isReal(const ::arma::cx_vec & vec);
bool isReal(const ::arma::cx_vec & vec, const double tolerance);

bool isReal(const ::arma::cx_mat & mat);
bool isReal(const ::arma::cx_mat & mat, const double tolerance);

}
}


#endif /* MATRIX_H */
