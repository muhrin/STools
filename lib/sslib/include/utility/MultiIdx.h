/*
 * MultiIdx.h
 *
 * An index in discreet n-space where n can be specified at runtime.
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef MULTI_IDX_H
#define MULTI_IDX_H

// INCLUDES /////////////////////////////////////////////
#include "SSLib.h"

#include <limits>
#include <stdexcept>

#include <boost/scoped_array.hpp>

#include "utility/PromotableType.h"


// FORWARD DECLARATIONS ////////////////////////////////////

namespace sstbx
{
namespace utility
{

template <typename IdxTyp = size_t>
class MultiIdx
{
public:

  MultiIdx() {}
	explicit MultiIdx(const size_t _dims);
	MultiIdx(const size_t _dims, const IdxTyp initialVal);
	// Copy constructor
	MultiIdx(const MultiIdx<IdxTyp> & toCopy);

	/** Reset back to the origin */
	void reset();

	void fill(IdxTyp value);

	IdxTyp min() const;
	IdxTyp max() const;
  IdxTyp sum() const;

	// Operators ///
	IdxTyp &		operator[](const size_t dim);
	const IdxTyp &	operator[](const size_t dim) const;

	// Same index type
	MultiIdx<IdxTyp> & operator =(const MultiIdx<IdxTyp> & rhs);
	MultiIdx<IdxTyp> operator+(const MultiIdx<IdxTyp> & rhs) const;
	MultiIdx<IdxTyp> operator-(const MultiIdx<IdxTyp> & rhs) const;
	MultiIdx<IdxTyp> & operator+=(const MultiIdx<IdxTyp> & rhs);
	MultiIdx<IdxTyp> & operator-=(const MultiIdx<IdxTyp> & rhs);
	bool operator ==(const MultiIdx<IdxTyp> & rhs) const;
	MultiIdx<int> operator<(const MultiIdx<IdxTyp> & rhs) const;

	// Different index type
	template <typename RhsTyp>
	MultiIdx<IdxTyp> & operator=(const MultiIdx<RhsTyp> & rhs);
	template <typename OutTyp, typename RhsTyp>
	MultiIdx<OutTyp> operator-(const MultiIdx<RhsTyp> & rhs) const;
	template <typename RhsTyp>
	MultiIdx<IdxTyp> & operator+=(const MultiIdx<RhsTyp> & rhs);
	template <typename RhsTyp>
	MultiIdx<IdxTyp> & operator-=(const MultiIdx<RhsTyp> & rhs);
	template <typename RhsTyp>
	MultiIdx<int> operator<(const MultiIdx<RhsTyp> & rhs) const;

	template <typename LhsTyp, typename RhsTyp>
	friend MultiIdx<typename PromoteType<LhsTyp, RhsTyp>::result > operator+(
		const MultiIdx<LhsTyp> & lhs,
		const MultiIdx<RhsTyp> & rhs);

	template <typename LhsTyp, typename RhsTyp>
	friend MultiIdx<typename PromoteType<LhsTyp, RhsTyp>::result > operator-(
		const MultiIdx<LhsTyp> & lhs,
		const MultiIdx<RhsTyp> & rhs);

	// Overload unary operators

	template <typename RhsTyp>
    friend MultiIdx<RhsTyp> operator-(const MultiIdx<RhsTyp> & rhs);

	template <typename RhsTyp>
	friend ::std::ostream & operator <<(
		::std::ostream &			os,
		const MultiIdx<RhsTyp> &	rhs);

	size_t dims() const;

  void setSize(const size_t newSize);

private:

  typedef ::boost::scoped_array<IdxTyp> IdxPtr;

  size_t    myDims;
	/** The current multidimensional index */
	IdxPtr	  myIdx;
};

// IMPLEMENTATION ///////////////////////

template <typename IdxTyp>
MultiIdx<IdxTyp>::MultiIdx(const size_t dims):
myDims(dims)
{
  myIdx.reset(new IdxTyp[myDims]);
	reset();
}

template <typename IdxTyp>
MultiIdx<IdxTyp>::MultiIdx(const size_t dims, const IdxTyp initialVal):
myDims(dims)
{
	myIdx.reset(new IdxTyp[myDims]);
	fill(initialVal);
}

template <typename IdxTyp>
MultiIdx<IdxTyp>::MultiIdx(const MultiIdx & toCopy):
myDims(toCopy.myDims)
{
	myIdx.reset(new IdxTyp[myDims]);
  memcpy(myIdx.get(), toCopy.myIdx.get(), sizeof(IdxTyp) * myDims);
}

template <typename IdxTyp>
void MultiIdx<IdxTyp>::reset()
{
	memset(myIdx.get(), 0, sizeof(IdxTyp) * myDims);
}

template <typename IdxTyp>
void MultiIdx<IdxTyp>::fill(IdxTyp value)
{
	for(size_t i = 0; i < myDims; ++i)
	{
		myIdx[i] = value;
	}
}

template <typename IdxTyp>
IdxTyp MultiIdx<IdxTyp>::min() const
{
	IdxTyp min = ::std::numeric_limits<IdxTyp>::max();
	for(size_t i = 0; i < myDims; ++i)
	{
		min = ::std::min(min, myIdx[i]);
	}
	return min;
}

template <typename IdxTyp>
IdxTyp MultiIdx<IdxTyp>::max() const
{
	IdxTyp max = ::std::numeric_limits<IdxTyp>::min();
	for(size_t i = 0; i < myDims; ++i)
	{
		max = ::std::max(max, myIdx[i]);
	}
	return max;
}

template <typename IdxTyp>
IdxTyp MultiIdx<IdxTyp>::sum() const
{
	IdxTyp sum = 0;
	for(size_t i = 0; i < myDims; ++i)
	{
		sum += myIdx[i];
	}
	return sum;
}

template <typename IdxTyp>
IdxTyp & MultiIdx<IdxTyp>::operator[](const size_t dim)
{
	SSE_ASSERT(dim < myDims);
	return myIdx[dim];
}

template <typename IdxTyp>
const IdxTyp & MultiIdx<IdxTyp>::operator [](const size_t dim) const
{
	SSE_ASSERT(dim < myDims);
	return myIdx[dim];
}

template <typename IdxTyp>
MultiIdx<IdxTyp> & MultiIdx<IdxTyp>::operator =(const MultiIdx<IdxTyp> & rhs)
{
	if(myDims != rhs.myDims)
	{
		throw ::std::logic_error("Multi index dimension mismatch");
	}
	memcpy(myIdx, rhs.myIdx, sizeof(IdxTyp) * myDims);
	return *this;
}

template <typename IdxTyp>
MultiIdx<IdxTyp> MultiIdx<IdxTyp>::operator+(const MultiIdx<IdxTyp> & rhs) const
{
	if(myDims != rhs.myDims)
	{
		throw ::std::logic_error("Multi index dimension mismatch");
	}
	MultiIdx<IdxTyp> result(myDims);
	for(size_t i = 0; i < myDims; ++i)
	{
		result[i] = myIdx[i] + rhs.myIdx[i];
	}
	return result;
}

template <typename IdxTyp>
MultiIdx<IdxTyp> MultiIdx<IdxTyp>::operator-(const MultiIdx<IdxTyp> & rhs) const
{
	if(myDims != rhs.myDims)
	{
		throw ::std::logic_error("Multi index dimension mismatch");
	}
	MultiIdx<IdxTyp> result(myDims);
	for(size_t i = 0; i < myDims; ++i)
	{
		result[i] = myIdx[i] - rhs.myIdx[i];
	}
	return result;
}

template <typename IdxTyp>
MultiIdx<IdxTyp> & MultiIdx<IdxTyp>::operator+=(const MultiIdx<IdxTyp> & rhs)
{
	if(myDims != rhs.myDims)
	{
		throw ::std::logic_error("Multi index dimension mismatch");
	}
	for(size_t i = 0; i < myDims; ++i)
	{
		myIdx[i] += rhs.myIdx[i];
	}
	return *this;
}

template <typename IdxTyp>
MultiIdx<IdxTyp> & MultiIdx<IdxTyp>::operator-=(const MultiIdx<IdxTyp> & rhs)
{
	if(myDims != rhs.myDims)
	{
		throw ::std::logic_error("Multi index dimension mismatch");
	}
	for(size_t i = 0; i < myDims; ++i)
	{
		myIdx[i] -= rhs.myIdx[i];
	}
	return *this;
}

template <typename IdxTyp>
bool MultiIdx<IdxTyp>::operator==(const MultiIdx<IdxTyp> & rhs) const
{
	if(myDims != rhs.myDims)
		return false;

	bool result = true;
	for(size_t i = 0; i < myDims; ++i)
	{
		if(myIdx[i] != rhs.myIdx[i])
		{
			result = false;
			break;
		}
	}
	return result;
}

template <typename IdxTyp>
MultiIdx<int>  MultiIdx<IdxTyp>::operator<(const MultiIdx<IdxTyp> & rhs) const
{
	if(myDims != rhs.myDims)
	{
		throw ::std::logic_error("Multi index dimension mismatch");
	}
	MultiIdx<int> result(myDims);
	for(size_t i = 0; i < myDims; ++i)
	{
		result.myIdx[i] = myIdx[i] < rhs.myIdx[i];
	}
	return result;
}

template <typename IdxTyp>
template <typename RhsTyp>
MultiIdx<IdxTyp> & MultiIdx<IdxTyp>::operator=(const MultiIdx<RhsTyp> & rhs)
{
	if(myDims != rhs.myDims)
	{
		throw ::std::logic_error("Multi index dimension mismatch");
	}
	for(size_t i = 0; i < rhs.myDims; ++i)
	{
		myIdx[i] = rhs[i];
	}
	return *this;
}

template <typename IdxTyp>
template <typename RhsTyp>
MultiIdx<IdxTyp> & MultiIdx<IdxTyp>::operator+=(const MultiIdx<RhsTyp> & rhs)
{
	if(myDims != rhs.myDims)
	{
		throw ::std::logic_error("Multi index dimension mismatch");
	}
	for(size_t i = 0; i < rhs.myDims; ++i)
	{
		myIdx[i] += rhs[i];
	}
	return *this;
}

template <typename IdxTyp>
template <typename RhsTyp>
MultiIdx<IdxTyp> & MultiIdx<IdxTyp>::operator-=(const MultiIdx<RhsTyp> & rhs)
{
	if(myDims != rhs.myDims)
	{
		throw ::std::logic_error("Multi index dimension mismatch");
	}
	for(size_t i = 0; i < rhs.myDims; ++i)
	{
		myIdx[i] -= rhs[i];
	}
	return *this;
}

template <typename IdxTyp>
template <typename RhsTyp>
MultiIdx<int> MultiIdx<IdxTyp>::operator<(const MultiIdx<RhsTyp> & rhs) const
{
	if(myDims != rhs.myDims)
	{
		throw ::std::logic_error("Multi index dimension mismatch");
	}
	MultiIdx<int> result(myDims);
	for(size_t i = 0; i < myDims; ++i)
	{
		result.myIdx[i] = myIdx[i] < rhs[i];
	}
	return result;
}

template <typename IdxTyp>
template <typename OutTyp, typename RhsTyp>
MultiIdx<OutTyp>  MultiIdx<IdxTyp>::operator-(const MultiIdx<RhsTyp> & rhs) const
{
	if(myDims != rhs.myDims)
	{
		throw ::std::logic_error("Multi index dimension mismatch");
	}
	MultiIdx<OutTyp> result(myDims);
	for(size_t i = 0; i < myDims; ++i)
	{
		result[i] = myIdx[i] - rhs[i];
	}
	return result;
}


template <typename IdxTyp>
size_t MultiIdx<IdxTyp>::dims() const
{
  return myDims;
}

template <typename IdxTyp>
void MultiIdx<IdxTyp>::setSize(const size_t newSize)
{
  if(newSize != myDims)
  {
    myDims = newSize;
    myIdx.reset(new IdxTyp[myDims]);
  }
}

// LONE OPERATORS //////////////////////////////////////

template <typename RhsTyp>
MultiIdx<RhsTyp> operator-(const MultiIdx<RhsTyp> & rhs)
{
	MultiIdx<RhsTyp> negation(rhs.myDims);
	for(size_t i = 0; i < rhs.myDims; ++i)
	{
		negation.myIdx[i] = -rhs.myIdx[i];
	}
	return negation;
}



template <typename LhsTyp, typename RhsTyp>
inline MultiIdx<typename PromoteType<LhsTyp, RhsTyp>::result > operator+(
	const MultiIdx<LhsTyp> & lhs,
	const MultiIdx<RhsTyp> & rhs)
{
	typedef typename PromoteType<LhsTyp, RhsTyp>::result eOutT;

	if(lhs.myDims != rhs.myDims)
	{
		throw ::std::logic_error("Multi index dimension mismatch");
	}
	const size_t myDims = lhs.myDims;
	MultiIdx<eOutT> result(myDims);
	for(size_t i = 0; i < myDims; ++i)
	{
		result.myIdx[i] = lhs.myIdx[i] + rhs.myIdx[i];
	}
	return result;
}

template <typename LhsTyp, typename RhsTyp>
inline MultiIdx<typename PromoteType<LhsTyp, RhsTyp>::result > operator-(
	const MultiIdx<LhsTyp> & lhs,
	const MultiIdx<RhsTyp> & rhs)
{
	typedef typename PromoteType<LhsTyp, RhsTyp>::result eOutT;

	if(lhs.myDims != rhs.myDims)
	{
		throw ::std::logic_error("Multi index dimension mismatch");
	}
	const size_t myDims = lhs.myDims;
	MultiIdx<eOutT> result(myDims);
	for(size_t i = 0; i < myDims; ++i)
	{
		result.myIdx[i] = lhs.myIdx[i] - rhs.myIdx[i];
	}
	return result;
}

template <typename RhsTyp>
inline ::std::ostream & operator <<(
	::std::ostream &			os,
	const MultiIdx<RhsTyp> &	rhs)
{
	for(size_t i = 0; i < rhs.myDims; ++i)
	{
		os << rhs.myIdx[i] << " ";
	}
	return os;
}


}}

#endif /* MULTI_IDX_H */

