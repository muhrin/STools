/*
 * Histogram.h
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef HISTOGRAM_H
#define HISTOGRAM_H

// INCLUDES ////////////
#include "SSLib.h"

#include <ostream>

// DEFINITION ///////////////////////

namespace sstbx {

// FORWARD DECLARATIONS ///////

namespace analysis {

class Histogram
{
  typedef ::std::vector<unsigned int> Bins;
public:
  typedef Bins::iterator iterator;
  typedef Bins::const_iterator const_iterator;

  Histogram(const double binWidth);

  void insert(const double value);
  void clear();
  size_t numBins() const;
  unsigned int getValue(const size_t binIndex) const;

  iterator begin();
  const_iterator begin() const;

  iterator end();
  const_iterator end() const;

  void print(::std::ostream & os) const;

private:

  void ensureEnoughBins(const unsigned int binIndex);
  size_t binIndex(const double value);
  int getFullestBin() const;


  const double myBinWidth;
  Bins myBins;

};

}
}

std::ostream & operator<<(std::ostream & os, const sstbx::analysis::Histogram & hist);

#endif /* HISTOGRAM_H */
