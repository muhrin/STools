/*
 * AnchorPointArrangement.cpp
 *
 *  Created on: Oct 17, 2013
 *      Author: Martin Uhrin
 */

#include "spl/analysis/AnchorPointArrangement.h"

#ifdef SSLIB_USE_CGAL

#include "spl/SSLibAssert.h"

namespace spl {
namespace analysis {

AnchorPoint::AnchorPoint(const size_t idx, const ::arma::vec2 & pos,
    const double maxDisplacement) :
    idx_(idx), anchorPos_(pos), pos_(pos), maxDisplacement_(maxDisplacement)
{
}

const ::arma::vec2 &
AnchorPoint::getAnchorPos() const
{
  return anchorPos_;
}

const ::arma::vec2 &
AnchorPoint::getPos() const
{
  return pos_;
}

void
AnchorPoint::setPos(const ::arma::vec2 & newPos)
{
    pos_ = newPos;
}

AnchorPoint::NeighbourIterator
AnchorPoint::neighboursBegin() const
{
  return neighbours_.begin();
}

AnchorPoint::NeighbourIterator
AnchorPoint::neighboursEnd() const
{
  return neighbours_.end();
}

double
AnchorPoint::getMaxDisplacement() const
{
  return maxDisplacement_;
}

size_t
AnchorPoint::numNeighbours() const
{
  return neighbours_.size();
}

size_t
AnchorPoint::idx() const
{
  return idx_;
}

void
AnchorPoint::addNeighbour(AnchorPoint * const neighbour)
{
  neighbours_.insert(neighbour);
}

bool
AnchorPoint::hasNeighbour(AnchorPoint * const neighbour) const
{
  return neighbours_.find(neighbour) != neighbours_.end();
}

void
AnchorPoint::swapNeighbours(AnchorPoint * const old,
    AnchorPoint * const newNeighbour)
{
  const AnchorPoint::Neighbours::iterator it = ::std::find(neighbours_.begin(),
      neighbours_.end(), old);

  SSLIB_ASSERT(it != neighbours_.end());

  neighbours_.erase(it);
  neighbours_.insert(newNeighbour);
}

void
AnchorPoint::clearNeighbours()
{
  neighbours_.clear();
}

}
}

#endif // SSLIB_USE_CGAL
