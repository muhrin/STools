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

// FORWARD DECLARES ////////////////////////////////

// DEFINES ////////////////////////////////////////

// FUNCTIONS ////////////////////////////////////////

namespace sstbx {
namespace math {

inline void seed(const unsigned int)
{
  srand(static_cast<unsigned int>(time(NULL)));
}

template <typename T>
struct Rand
{
private:
  Rand(); // non constructible
};

template <typename T>
inline T rand()
{
  return Rand<T>::get();
}

template <typename T>
inline T rand(const T to)
{
  return Rand<T>::get(to);
}

template <typename T>
inline T rand(const T from, const T to)
{
  return Rand<T>::get(from, to);
}

// Specialisations
// TODO: Make these use boost random as this method doesn't generate
// uniformly distributed numbers
template <>
struct Rand<int>
{
  static int get(const int to)
  {
    return ::rand() % to;
  }
  static int get(const int from, const int to)
  {
    return get(to - from) + from;
  }
};

template <>
struct Rand<unsigned int>
{
  static unsigned int get(const unsigned int to)
  {
    return static_cast<unsigned int>(::rand() % static_cast<int>(to));
  }
  static unsigned int get(const unsigned int from, const unsigned int to)
  {
    return get(to - from) + from;
  }
};

template <>
struct Rand<long unsigned int>
{
  static long unsigned int get(const long unsigned int to)
  {
    return static_cast<long unsigned int>(::rand() % static_cast<long int>(to));
  }
  static long unsigned int get(const long unsigned int from, const long unsigned int to)
  {
    return get(to - from) + from;
  }

};

template <>
struct Rand<double>
{
  static double get()
  {
    return static_cast<double>(::rand()) / static_cast<double>(RAND_MAX);
  }
  static double get(const double to)
  {
    return get() * to;
  }
  static double get(const double from, const double to)
  {
    return get(to - from) + from;
  }
};

}
}


#endif /* RANDOM_DETAIL_H */
