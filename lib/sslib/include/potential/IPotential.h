/*
 * IPotential.h
 *
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef I_POTENTIAL_H
#define I_POTENTIAL_H

// INCLUDES /////////////////////////////////////////////
#include "PotentialData.h"

#include <boost/optional.hpp>
#include <boost/smart_ptr.hpp>

#include <common/AtomSpeciesId.h>

// FORWARD DECLARATIONS ////////////////////////////////////
namespace sstbx {
namespace common {
class Structure;
}
namespace potential {
class IPotentialInfo;
class IPotentialEvaluator;
}
}

namespace sstbx {
namespace potential {

class IPotential
{
public:

	virtual ~IPotential() {}

	virtual const ::std::string & getName() const = 0;

  virtual ::boost::optional<double> getPotentialRadius(const ::sstbx::common::AtomSpeciesId::Value id) const = 0;

  virtual ::boost::shared_ptr<IPotentialEvaluator>
    createEvaluator(const sstbx::common::Structure & structure) const = 0;
};

}}

#endif /* I_POTENTIAL_H */
