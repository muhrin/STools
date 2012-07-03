/*
 * HeterogeneousMap.h
 *
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef HETEROGENEOUS_MAP_H
#define HETEROGENEOUS_MAP_H

// INCLUDES /////////////////////////////////////////////

#include <map>

#include <boost/any.hpp>
#include <boost/iterator/iterator_facade.hpp>
#include <boost/utility.hpp>


// FORWARD DECLARATIONS ////////////////////////////////////

namespace sstbx
{
namespace utility
{

struct KeySupertype : ::boost::noncopyable
{
protected:
	// Can't create an instance of this type
	KeySupertype() {}
};

template <typename Type>
struct Key : public KeySupertype
{
public:
  typedef Type ValueType;

  Key() {}
};


class HeterogeneousMap
{
private:
  typedef ::std::map<const KeySupertype *, ::boost::any> AnyMap;
public:

 
  // Capacity methods
  bool empty() const;
  size_t size() const;
  size_t max_size() const;

	template<typename T>
	bool insert(const Key<T> & key, T value);

	template<typename T>
	T * find(const Key<T> & key);

	template<typename T>
	const T * find(const Key<T> & key) const;

	template<typename T>
	size_t erase(const Key<T> & key);

  void clear();

private:


	AnyMap myAnyMap;
};

// IMPLEMENTATION /////////////////////////////////////


template <typename T>
bool HeterogeneousMap::insert(const Key<T> & key, T value)
{
  ::std::pair<AnyMap::iterator, bool> result = myAnyMap.insert(AnyMap::value_type(&key, value));

	//T & out =	result.second ?
	//	value :
	//	*::boost::any_cast<T>(&result.first->second);

  // Overriding default map behaviour and forcing value in even though
  // a value may already exist TOOD: FIX THIS!
  result.first->second = value;

	return result.second;
}

template <typename T>
T * HeterogeneousMap::find(const Key<T> & key)
{
	const AnyMap::iterator it = myAnyMap.find(&key);

	if(it == myAnyMap.end())
		return NULL;

	return ::boost::any_cast<T>(&it->second);
}

template <typename T>
const T * HeterogeneousMap::find(const Key<T> & key) const
{
	const AnyMap::const_iterator it = myAnyMap.find(&key);

	if(it == myAnyMap.end())
		return NULL;

	return ::boost::any_cast<T>(&it->second);
}

template <typename T>
size_t HeterogeneousMap::erase(const Key<T> & key)
{
	const AnyMap::iterator it = myAnyMap.find(&key);

	if(it == myAnyMap.end())
		return 0;

	myAnyMap.erase(it);
  return 1;
}

}
}

#endif /* HETEROGENEOUS_MAP_H */
