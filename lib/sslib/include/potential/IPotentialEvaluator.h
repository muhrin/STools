/*
 * IPotentialEvaluator.h
 *
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef I_POTENTIAL_EVALUATOR_H
#define I_POTENTIAL_EVALUATOR_H

// INCLUDES /////////////////////////////////////////////
#include "StandardData.h"

// FORWARD DECLARATIONS ////////////////////////////////////
namespace sstbx
{
namespace potential
{
template <typename FloatTyp>
struct StandardData;
}
}
namespace sstbx
{
namespace potential
{

class IPotentialEvaluator
{
public:

  virtual ::boost::shared_ptr<StandardData<double> > getData() = 0;

	virtual void evalPotential() = 0;

};

}}

#endif /* I_POTENTIAL_EVALUATOR_H */
