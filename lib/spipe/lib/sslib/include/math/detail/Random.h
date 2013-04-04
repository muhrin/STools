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

#include <boost/version.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/normal_distribution.hpp>
#if BOOST_VERSION / 100000 <= 1 && BOOST_VERSION / 100 % 1000 <= 42
#  define SSLIB_USE_BOOST_OLD_RANDOM
#  include <boost/random/uniform_int.hpp>
#  include <boost/random/uniform_real.hpp>
#  include <boost/random/variate_generator.hpp>
#else
#  include <boost/random/uniform_int_distribution.hpp>
#  include <boost/random/uniform_real_distribution.hpp>
#endif

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

extern boost::mt19937 mt19937;

// Specialisations
// TODO: Make these use boost random as this method doesn't generate
// uniformly distributed numbers
template <>
struct Rand<int>
{
  static int getUniform(const int to)
  {
#ifdef SSLIB_USE_BOOST_OLD_RANDOM
    const ::boost::uniform_int<> dist(0, to - 1);
    ::boost::variate_generator<boost::mt19937&, boost::uniform_int<> > gen(mt19937, dist);
    return gen();
#else
    const ::boost::random::uniform_int_distribution<> dist(0, to - 1);
    return dist(mt19937);
#endif
  }
  static int getUniform(const int from, const int to)
  {
#ifdef SSLIB_USE_BOOST_OLD_RANDOM
    const ::boost::uniform_int<> dist(from, to - 1);
    ::boost::variate_generator<boost::mt19937&, boost::uniform_int<> > gen(mt19937, dist);
    return gen();
#else
    const ::boost::random::uniform_int_distribution<> dist(from, to - 1);
    return dist(mt19937);
#endif
  }
};

template <>
struct Rand<unsigned int>
{
  static unsigned int getUniform(const unsigned int to)
  {
    SSLIB_ASSERT(to > 0);

#ifdef SSLIB_USE_BOOST_OLD_RANDOM
    const ::boost::uniform_int<unsigned int> dist(0, to - 1);
    ::boost::variate_generator<boost::mt19937&, boost::uniform_int<unsigned int> > gen(mt19937, dist);
    return gen();
#else
    const ::boost::random::uniform_int_distribution<unsigned int> dist(0, to - 1);
    return dist(mt19937);
#endif
  }
  static unsigned int getUniform(const unsigned int from, const unsigned int to)
  {
    SSLIB_ASSERT(to > from);

#ifdef SSLIB_USE_BOOST_OLD_RANDOM
    const ::boost::uniform_int<unsigned int> dist(from, to - 1);
    ::boost::variate_generator<boost::mt19937&, boost::uniform_int<unsigned int> > gen(mt19937, dist);
    return gen();
#else
    const ::boost::random::uniform_int_distribution<unsigned int> dist(from, to - 1);
    return dist(mt19937);
#endif
  }
};

template <>
struct Rand<long unsigned int>
{
  static long unsigned int getUniform(const long unsigned int to)
  {
    SSLIB_ASSERT(to > 0);

#ifdef SSLIB_USE_BOOST_OLD_RANDOM
    const ::boost::uniform_int<long unsigned int> dist(0, to - 1);
    ::boost::variate_generator<boost::mt19937&, boost::uniform_int<long unsigned int> > gen(mt19937, dist);
    return gen();
#else
    const ::boost::random::uniform_int_distribution<long unsigned int> dist(0, to - 1);
    return dist(mt19937);
#endif
  }
  static long unsigned int getUniform(const long unsigned int from, const long unsigned int to)
  {
    SSLIB_ASSERT(to > from);

#ifdef SSLIB_USE_BOOST_OLD_RANDOM
    const ::boost::uniform_int<long unsigned int> dist(from, to - 1);
    ::boost::variate_generator<boost::mt19937&, boost::uniform_int<long unsigned int> > gen(mt19937, dist);
    return gen();
#else
    const ::boost::random::uniform_int_distribution<long unsigned int> dist(from, to - 1);
    return dist(mt19937);
#endif
  }
};

template <>
struct Rand<double>
{
  static ::boost::normal_distribution<> normal;
#ifdef SSLIB_USE_BOOST_OLD_RANDOM
  static const ::boost::uniform_real<> uniform;
  static ::boost::variate_generator< ::boost::mt19937 &, ::boost::uniform_real<> > uniformGenerator;
  static ::boost::variate_generator< ::boost::mt19937 &, ::boost::normal_distribution<> > normalGenerator;
#else
  static const ::boost::random::uniform_real_distribution<> uniform;
#endif

  static double getUniform()
  {
#ifdef SSLIB_USE_BOOST_OLD_RANDOM
    return uniformGenerator();
#else
    return uniform(mt19937);
#endif
  }
  static double getUniform(const double to)
  {
#ifdef SSLIB_USE_BOOST_OLD_RANDOM
    return uniformGenerator() * to;
#else
    return uniform(mt19937) * to;
#endif
  }
  static double getUniform(const double from, const double to)
  {
    return getUniform(to - from) + from;
  }
  static double getNormal()
  {
#ifdef SSLIB_USE_BOOST_OLD_RANDOM
    return normalGenerator();
#else
    return normal(mt19937);
#endif
  }
  static double getNormal(const double mean, const double variance)
  {
    ::boost::normal_distribution<> normal(mean, variance);
#ifdef SSLIB_USE_BOOST_OLD_RANDOM
    ::boost::variate_generator<boost::mt19937&, ::boost::normal_distribution<> >
      normalGen(mt19937, normal);
    normalGen();
#else
    return normal(mt19937);
#endif
  }
};

} // namespace detail

inline void seed(const unsigned int randSeed)
{
  ::std::srand(randSeed);
  detail::mt19937.seed(randSeed);
}

inline void seed()
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
