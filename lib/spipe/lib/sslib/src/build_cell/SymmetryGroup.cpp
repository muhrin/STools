/*
 * SymmetryGroup.cpp
 *
 *  Created on: Nov 10, 2011
 *      Author: Martin Uhrin
 */

// INCLUEDES /////////////
#include "build_cell/SymmetryGroup.h"

#include <complex>
#include <vector>

#include <boost/foreach.hpp>

#include "SSLibAssert.h"
#include "utility/StableComparison.h"

namespace sstbx {
namespace build_cell {

SymmetryGroup::SymmetryGroup()
{
  // Every symmetry group has the identity operator
  mySymOps.push_back(::arma::eye< ::arma::mat>(4, 4));
}

size_t SymmetryGroup::numOps() const
{
  return mySymOps.size();
}

const ::arma::mat44 & SymmetryGroup::getOp(const size_t idx) const
{
  return mySymOps[idx];
}

SymmetryGroup::OperatorsIterator
SymmetryGroup::beginOperators() const
{
  return mySymOps.begin();
}

SymmetryGroup::OperatorsIterator
SymmetryGroup::endOperators() const
{
  return mySymOps.end();
}

SymmetryGroup::Multiplicities
SymmetryGroup::getMultiplicities() const
{
  Multiplicities mults;
  mults.reserve(myInvariantsMap.size());
  BOOST_FOREACH(InvariantsMap::const_reference entry, myInvariantsMap)
  {
    mults.push_back(entry.first);
  }
  return mults;
}

const SymmetryGroup::EigenvectorsOpsList *
SymmetryGroup::getEigenvectorsOpsList(const unsigned int multiplicity) const
{
  const InvariantsMap::const_iterator it = myInvariantsMap.find(multiplicity);
  if(it == myInvariantsMap.end())
    return NULL;

  return &(it->second);
}

void SymmetryGroup::generateMultiplicityEigenvectors()
{
  typedef ::std::vector<EigenvectorsList> EigenvectorsLists;
  typedef ::std::vector<EigenvectorsLists> EigenvectorsListsList;
  
  ::arma::cx_vec eigval;
  ::arma::cx_mat eigvec;

  EigenvectorsListsList eigenvectorsListsList(numOps());
  ::arma::vec3 invariantAxis;
  // NOTE: Always skip over first (identity) operator
  for(size_t op = 1; op < numOps(); ++op)
  {
    if(eig_gen(eigval, eigvec, mySymOps[op].submat(0, 0, 2, 2)))
    {
      // Check each of the eigenvectors and values for invariant axes
      for(size_t j = 0; j < 3; ++j)
      {
        if(getInvariantAxis(invariantAxis, eigval(j), eigvec.col(j)))
        {
          EigenvectorsList list;
          list.push_back(invariantAxis);
          eigenvectorsListsList[op].push_back(list);
        }
      }
      // Now create the combination of two (a plane) if possible
      if(eigenvectorsListsList[op].size() == 2)
      {
        EigenvectorsList list(2);
        list[0] = eigenvectorsListsList[op][0][0];
        list[1] = eigenvectorsListsList[op][1][0];
        eigenvectorsListsList[op].push_back(list);
      }
    }
  }

  // Now reduce the eigenvectors down to the unique set reducing the multiplicity
  // each time a duplicate is found
  unsigned int multiplicity;
  for(size_t op1 = 1; op1 < numOps() - 1; ++op1)
  {
    for(EigenvectorsLists::iterator vecs1It = eigenvectorsListsList[op1].begin(),
      vecs1End = eigenvectorsListsList[op1].end(); vecs1It != vecs1End; /*increment in body*/)
    {
      multiplicity = numOps() - 1;
      EigenvectorsOps eigenOps;
      eigenOps.first = *vecs1It;
      // Apply all ops execpt those that we explicitly set to false
      eigenOps.second.insert(eigenOps.second.begin(), numOps(), true);
      eigenOps.second[op1] = false;
      for(size_t op2 = op1 + 1; op2 < numOps(); ++op2)
      {
        for(EigenvectorsLists::iterator vecs2It = eigenvectorsListsList[op2].begin();
          vecs2It != eigenvectorsListsList[op2].end(); /*increment in body*/)
        {
          if(eigenvectorsListSame(*vecs1It, *vecs2It))
          {
            --multiplicity;
            eigenOps.second[op2] = false; // Don't apply this operator
            vecs2It = eigenvectorsListsList[op2].erase(vecs2It);
          }
          else
            ++vecs2It;
        } // for vecs2
      } // for op2

      myInvariantsMap[multiplicity].push_back(eigenOps);
      ++vecs1It;
    } // for vecs1
  } // for op1

  // Finally create the multiplicity entry for full symmetry
  EigenvectorsOpsList & opsList = myInvariantsMap[numOps()];
  opsList.resize(1);
  EigenvectorsOps & eigenOps = opsList[0];
  eigenOps.second.insert(eigenOps.second.begin(), numOps(), true);
  ::arma::vec3 vec;
  vec << 1.0 << 0.0 << 0.0;
  eigenOps.first.push_back(vec);
  vec(0) = 0.0;
  vec(1) = 1.0;
  eigenOps.first.push_back(vec);
  vec(1) = 0.0;
  vec(2) = 1.0;
  eigenOps.first.push_back(vec);
}


bool SymmetryGroup::getInvariantAxis(
  ::arma::vec3 & invariantAxis,
  const ::std::complex<double> & eigVal,
  const ::arma::cx_vec & eigVec
) const
{
  if(::std::abs(::std::imag(eigVal)) != 0.0)
    return false;

  invariantAxis.set_size(eigVec.n_rows);
  for(size_t i = 0; i < eigVec.n_rows; ++i)
  {
    // Need abs as the value can be +/- 0
    if(::std::abs(::std::imag(eigVec(i))) != 0.0)
      return false;
    invariantAxis(i) = ::std::real(eigVec(i));
  }
  return true;
}

bool SymmetryGroup::eigenvectorsListSame(
  const EigenvectorsList & vecs1,
  const EigenvectorsList & vecs2) const
{
  const size_t numVecs1 = vecs1.size();
  const size_t numVecs2 = vecs2.size();
  if(numVecs1 != numVecs2)
    return false;

  size_t numMatched = 0;
  // Can use the fact that vecs1 & 2 don't contain duplicates
  for(size_t i = 0; i < numVecs1; ++i)
  {
    for(size_t j = 0; j < numVecs2; ++j)
    {
      if(vecsSame(vecs1[i], vecs2[j]))
        ++numMatched;
    }
  }
  return numMatched == numVecs1;
}

bool SymmetryGroup::vecsSame(const Eigenvector & e1, const Eigenvector & e2) const
{
  SSLIB_ASSERT(e1.n_rows == e2.n_rows);

  for(size_t i = 0; i < e1.n_rows; ++i)
  {
    if(!utility::StableComp::eq(e1(i), e2(i)))
      return false;
  }
  return true;
}

}
}
