/*
 * VectorAnchorArrangementOutputter.h
 *
 *  Created on: Nov 11, 2013
 *      Author: Martin Uhrin
 */

#ifndef VECTOR_ANCHOR_ARRANGEMENT_OUTPUTTER_H
#define VECTOR_ANCHOR_ARRANGEMENT_OUTPUTTER_H

// INCLUDES ////////////
#include "spl/SSLib.h"

#ifdef SPL_WITH_CGAL

#include <map>

#include <spl/analysis/AnchorArrangementOutputter.h>

// DEFINITION ///////////////////////

namespace spl {

// FORWARD DECLARATIONS ///////

namespace analysis {

template< typename Map>
  class VectorAnchorArrangementOutputter : public AnchorArrangementOutputter<
      Map>
  {
  public:
    typedef typename AnchorArrangementOutputter< Map>::InfoMap InfoMap;
    typedef typename AnchorArrangementOutputter< Map>::Arrangement Arrangement;

    virtual
    ~VectorAnchorArrangementOutputter()
    {
    }

    virtual bool
    outputArrangement(const Arrangement & arrangement) const;
    virtual bool
    outputArrangement(const Arrangement & arrangement,
        const InfoMap & labelInfo) const;
  };

}
}

#include "spl/analysis/detail/VectorAnchorArrangementOutputter.h"

#endif // SPL_WITH_CGAL
#endif /* VECTOR_ANCHOR_ARRANGEMENT_OUTPUTTER_H */
