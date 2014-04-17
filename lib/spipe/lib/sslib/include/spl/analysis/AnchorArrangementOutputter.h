/*
 * AnchorArrangementOutputter.h
 *
 *  Created on: Nov 11, 2013
 *      Author: Martin Uhrin
 */

#ifndef ANCHOR_ARRANGEMENT_OUTPUTTER_H
#define ANCHOR_ARRANGEMENT_OUTPUTTER_H

// INCLUDES ////////////
#include "spl/SSLib.h"

#ifdef SPL_WITH_CGAL

#include <map>

// DEFINITION ///////////////////////

namespace spl {

// FORWARD DECLARATIONS ///////

namespace analysis {

template< typename Map>
  class AnchorArrangementOutputter
  {
  public:
    typedef typename Map::Arrangement Arrangement;
    typedef std::map< typename Map::Label, ::std::string> InfoMap;

    virtual
    ~AnchorArrangementOutputter()
    {
    }

    virtual bool
    outputArrangement(const Arrangement & map) const = 0;
    virtual bool
    outputArrangement(const Arrangement & map,
        const InfoMap & labelInfo) const = 0;
  };

}
}

#endif // SPL_WITH_CGAL
#endif /* ANCHOR_ARRANGEMENT_OUTPUTTER_H */
