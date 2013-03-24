/*
 * Random.h
 *
 *  Created on: Aug 22, 2011
 *      Author: Martin Uhrin
 */


#ifndef RANDOM_DETAIL_H
#define RANDOM_DETAIL_H

// INCLUDES ///////////////////////////////////////
#include <stdlib.h>
#include <time.h>

#include <boost/random/mersenne_twister.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/uniform_int_distribution.hpp>
#include <boost/random/uniform_real_distribution.hpp>

#include "SSLibAssert.h"

// FORWARD DECLARES ////////////////////////////////

// DEFINES ////////////////////////////////////////

// FUNCTIONS ////////////////////////////////////////

namespace sstbx {
namespace math {
namespace detail {


template <typename T>
struct Rand
{
private:
  Rand(); // non constructible
};

extern boost::random::mt19937 mt19937;

// Specialisations
// TODO: Make these use boost random as this method doesn't generate
// uniformly distributed numbers
template <>
struct Rand<int>
{
  static int getUniform(const int to)
  {
    const ::boost::random::uniform_int_distribution<> dist(0, to - 1);
    return dist(mt19937);
  }
  static int getUniform(const int from, const int to)
  {
    const ::boost::random::uniform_int_distribution<> dist(from, to - 1);
    return dist(mt19937);
  }
};

template <>
struct Rand<unsigned int>
{
  static unsigned int getUniform(const unsigned int to)
  {
    SSLIB_ASSERT(to > 0);

    const ::boost::random::uniform_int_distribution<unsigned int> dist(0, to - 1);
    return dist(mt19937);
  }
  static unsigned int getUniform(const unsigned int from, const unsigned int to)
  {
    SSLIB_ASSERT(to > from);

    const ::boost::random::uniform_int_distribution<unsigned int> dist(from, to - 1);
    return dist(mt19937);
  }
};

template <>
struct Rand<long unsigned int>
{
  static long unsigned int getUniform(const long unsigned int to)
  {
    SSLIB_ASSERT(to > 0);

    const ::boost::random::uniform_int_distribution<long unsigned int> dist(0, to - 1);
    return dist(mt19937);
  }
  static long unsigned int getUniform(const long unsigned int from, const long unsigned int to)
  {
    SSLIB_ASSERT(to > from);

    const ::boost::random::uniform_int_distribution<long unsigned int> dist(from, to - 1);
    return dist(mt19937);
  }

};

template <>
struct Rand<double>
{
  static const ::boost::random::uniform_real_distribution<> uniform;
  static ::boost::random::normal_distribution<> normal;
  static double getUniform()
  {
    return uniform(mt19937);
  }
  static double getUniform(const double to)
  {
    return uniform(mt19937) * to;
  }
  static double getUniform(const double from, const double to)
  {
    return getUniform(to - from) + from;
  }
  static double getNormal()
  {
    return normal(mt19937);
  }
  static double getNormal(const double mean, const double variance)
  {
    ::boost::random::normal_distribution<> normal(mean, variance);
    return normal(mt19937);
  }
};

} // namespace detail

inline void seed(const unsigned int)
{
  ::std::srand(static_cast<unsigned int>(time(NULL)));
  detail::mt19937.seed(static_cast<unsigned int>(time(NULL)));
}

template <typename T>
inline T randu()
{
  return detail::Rand<T>::getUniform();
}

template <typename T>
inline T randu(const T to)
{
  return detail::Rand<T>::getUniform(to);
}

template <typename T>
inline T randu(const T from, const T to)
{
  return detail::Rand<T>::getUniform(from, to);
}

template <typename T>
inline T randn()
{
  return detail::Rand<T>::getNormal();
}
template <typename T>
inline T randn(const T mean, const T variance)
{
  return detail::Rand<T>::getNormal(mean, variance);
}


}
}


#endif /* RANDOM_DETAIL_H */
