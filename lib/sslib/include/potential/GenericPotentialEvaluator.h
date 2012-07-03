/*
 * GenericPotentialEvaluator.h
 *
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef GENERIC_POTENTIAL_EVALUATOR_H
#define GENERIC_POTENTIAL_EVALUATOR_H

// INCLUDES /////////////////////////////////////////////
#include "IPotentialEvaluator.h"

#include <boost/shared_ptr.hpp>

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

template <class PotTyp, class DataTyp>
class GenericPotentialEvaluator : public IPotentialEvaluator
{
public:

  GenericPotentialEvaluator(const PotTyp & potential, ::boost::shared_ptr<DataTyp> & data);

  virtual ::boost::shared_ptr<StandardData<double> > getData();

	virtual bool evalPotential();

private:

  ::boost::shared_ptr<DataTyp>  myData;
  const PotTyp &                myPotential;

};

// IMPLEMENTATION //////////////////

template <class PotTyp, class DataTyp>
GenericPotentialEvaluator<PotTyp, DataTyp>::GenericPotentialEvaluator(const PotTyp & potential, ::boost::shared_ptr<DataTyp> & data):
myPotential(potential),
myData(data)
{}

template <class PotTyp, class DataTyp>
::boost::shared_ptr<StandardData<double> > GenericPotentialEvaluator<PotTyp, DataTyp>::getData()
{
  return myData;
}

template <class PotTyp, class DataTyp>
bool GenericPotentialEvaluator<PotTyp, DataTyp>::evalPotential()
{
  return myPotential.evaluate(*myData);
}

}
}

#endif /* GENERIC_POTENTIAL_EVALUATOR_H */
