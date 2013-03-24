/*
 * SymmetryGroup.h
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef SYMMETRY_GROUP_H
#define SYMMETRY_GROUP_H

// INCLUDES ////////////
#include "SSLib.h"

#include <map>
#include <set>
#include <vector>

#include <armadillo>

// DEFINITION ///////////////////////

namespace sstbx {
namespace build_cell {

class SymmetryGroup
{
  typedef ::std::vector< ::arma::mat44> SymOps;
  typedef ::arma::vec3 Eigenvector;

public:
  typedef ::std::vector<bool> OpMask;
  typedef ::std::vector<Eigenvector> EigenvectorsList;
  typedef SymOps::const_iterator OperatorsIterator;
  typedef ::std::vector<unsigned int> Multiplicities;
  typedef ::std::pair<EigenvectorsList, OpMask> EigenvectorsOps;
  typedef ::std::vector<EigenvectorsOps> EigenvectorsOpsList;

  SymmetryGroup();
  size_t numOps() const;
  const ::arma::mat44 & getOp(const size_t idx) const;
  OperatorsIterator beginOperators() const;
  OperatorsIterator endOperators() const;
  
  Multiplicities getMultiplicities() const;

  const EigenvectorsOpsList * getEigenvectorsOpsList(const unsigned int multiplicity) const;

protected:
  typedef ::std::map<unsigned int, EigenvectorsOpsList> InvariantsMap;


  void generateMultiplicityEigenvectors();
  bool getInvariantAxis(
    ::arma::vec3 & invariantAxis,
    const ::std::complex<double> & eigVal,
    const ::arma::cx_vec & eigVec
  ) const;
  bool eigenvectorsListSame(const EigenvectorsList & vecs1, const EigenvectorsList & vecs2) const;
  bool vecsSame(const Eigenvector & e1, const Eigenvector & e2) const;

  SymOps mySymOps;
  InvariantsMap myInvariantsMap;
};

}
}


#endif /* SYMMETRY_GROUP_H */
