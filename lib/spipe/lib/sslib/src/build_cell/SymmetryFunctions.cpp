/*
 * SymmetryFunctions.h
 *
 *  Created on: Nov 10, 2011
 *      Author: Martin Uhrin
 */


// INCLUDES /////////////////
#include "build_cell/SymmetryFunctions.h"

#include "SSLibAssert.h"

#include "math/Random.h"

namespace sstbx {
namespace build_cell {
namespace symmetry {

::std::vector<unsigned int> generateMultiplicities(const unsigned int numAtoms, const unsigned int numSymOps)
{
  ::std::vector<unsigned int> possibleMultiplicities;

  // First find out what the possible multiplicities are for this number of symmetry operations
  // i.e. the integers that can divide numSymOps
  possibleMultiplicities.push_back(1); // Can always have 1
  for(unsigned int i = 2; i <= numSymOps; ++i)
  {
    if(numSymOps % i == 0)
      possibleMultiplicities.push_back(i);
  }

  return generateMultiplicities(numAtoms, possibleMultiplicities);
}

::std::vector<unsigned int>
generateMultiplicities(
  const unsigned int numAtoms,
  const ::std::vector<unsigned int>  & possibleMultiplicities)
{
  const int MAX_ATTEMPTS = 50000;
  const int MAX_FACTOR_ATTEMPTS = 1000;
  SSLIB_ASSERT(MAX_ATTEMPTS > MAX_FACTOR_ATTEMPTS);

  ::std::vector<unsigned int> multiplicities;

  // First try the simple case of the highest factor
  if(numAtoms % possibleMultiplicities.back() == 0)
  {
    // Great, use that
    multiplicities.insert(
      multiplicities.begin(),
      numAtoms / possibleMultiplicities.back(), // how many to insert
      possibleMultiplicities.back()             // what to insert
    );
    return multiplicities;
  }
  else
  {
    // Have to find combination of multiplicities that will sum to numAtoms
    unsigned int multiplicitiesSum;
    unsigned int maxMultiplicities = numAtoms / possibleMultiplicities.back() + 1; // Start by allowing one more than the minumum
    for(int i = 0; i < MAX_ATTEMPTS; ++i)
    {
      multiplicitiesSum = 0;
      multiplicities.clear();
      do
      {
        // Choose a random symmetry factor and add it to the list
        const size_t multIdx = math::rand(possibleMultiplicities.size());
        multiplicities.push_back(possibleMultiplicities[multIdx]);
        multiplicitiesSum += multiplicities.back();

      } while(multiplicitiesSum < numAtoms && multiplicities.size() <= maxMultiplicities);

      // How did we do?
      if(multiplicitiesSum == numAtoms && multiplicities.size() <= maxMultiplicities)
        return multiplicities;

      if(i % MAX_FACTOR_ATTEMPTS == 0)
        ++maxMultiplicities;
    }
  }
  // Couldn't do it.  Return empty multiplicities vector
  return multiplicities;
}

}
}
}
