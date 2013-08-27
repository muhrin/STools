/*
 * RunningStats.h
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef RUNNING_STATS_H
#define RUNNING_STATS_H

// INCLUDES ////////////
#include "spl/SSLib.h"

// DEFINITION ///////////////////////

namespace spl {
namespace math {

class RunningStats
{
public:

  RunningStats();

  void insert(const double x);

  template <class InputIterator>
  void insert(InputIterator first, InputIterator last);

  unsigned int num() const;

  double min() const;
  double max() const;

  double mean() const;
  double sum() const;
  double sqSum() const;
  double rms() const;

private:
  unsigned int myNum;
  double mySum;
  double mySqSum;
  double myMin;
  double myMax;
};

template <class InputIterator>
void RunningStats::insert(InputIterator first, InputIterator last)
{
  for(InputIterator it = first; it != last; ++it)
    insert(*it);
}

}
}

#endif /* RUNNING_STATS_H */
