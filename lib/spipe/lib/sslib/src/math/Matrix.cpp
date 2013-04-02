/*
 * Matrix.cpp
 *
 *  Created on: Nov 10, 2011
 *      Author: Martin Uhrin
 */

// INCLUEDES /////////////

#include "SSLibAssert.h"
#include "math/Matrix.h"
#include "utility/StableComparison.h"

#include <limits>

namespace sstbx {
namespace math {

namespace comp = utility::StableComp;

void normalise(::arma::vec & vec)
{
  vec = vec / sqrt(::arma::dot(vec, vec));
}

::arma::vec normaliseCopy(const ::arma::vec & vec)
{
  ::arma::vec copy(vec);
  normalise(copy);
  return copy;
}

bool equals(const ::arma::mat & a, const ::arma::mat & b)
{
  return equals(a, b, 1e3 * ::std::numeric_limits<double>::epsilon());
}

bool equals(const ::arma::mat & a, const ::arma::mat & b, const double tolerance)
{
  if(a.n_rows != b.n_rows || a.n_cols != b.n_cols)
    return false;

  const ::arma::mat diff(::arma::abs(a - b));
  return comp::eq(diff.max(), 0.0, tolerance);
}

bool isZero(const ::arma::mat & mat)
{
  return isZero(mat, 1e3 * ::std::numeric_limits<double>::epsilon());
}

bool isZero(const ::arma::mat & mat, const double tolerance)
{
  const ::arma::mat abs(::arma::abs(mat));
  return comp::eq(abs.max(), 0.0, tolerance);
}

bool isSpannedBy(const ::arma::mat & superspace, const ::arma::mat & subspace)
{
  SSLIB_ASSERT(superspace.n_rows == subspace.n_rows);
  SSLIB_ASSERT(superspace.n_cols >= subspace.n_cols);

  // For the superspace to span the subspace there must be a non-zero
  // solution to the equation A * X = B
  bool isSpanned = false;
  ::arma::mat x;
  if(::arma::solve(x, superspace, subspace))
    isSpanned = !isZero(x);

  return isSpanned;
}

bool isReal(const ::arma::cx_vec & vec)
{
  return isReal(vec, 1e3 * ::std::numeric_limits<double>::epsilon());
}

bool isReal(const ::arma::cx_vec & vec, const double tolerance)
{
  const ::arma::vec im(::arma::imag(vec));
  return comp::eq(im.max(), 0.0, tolerance);
}

bool isReal(const ::arma::cx_mat & mat)
{
  return isReal(mat, 1e3 * ::std::numeric_limits<double>::epsilon());
}

bool isReal(const ::arma::cx_mat & mat, const double tolerance)
{
  const ::arma::vec im(::arma::imag(mat));
  return comp::eq(im.max(), 0.0, tolerance);
}

}
}
