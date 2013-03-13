/*
 * GenShapeFactory.h
 *
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef GEN_SHAPE_FACTORY_H
#define GEN_SHAPE_FACTORY_H

// INCLUDES /////////////////////////////////////////////
#include "SSLib.h"

#include "build_cell/IGeneratorShape.h"
#include "utility/HeterogeneousMap.h"

// FORWARD DECLARATIONS ////////////////////////////////////

namespace sstbx {
namespace factory {

class GenShapeFactory : ::boost::noncopyable
{
public:
  typedef UniquePtr<build_cell::IGeneratorShape>::Type GenShapePtr;
  typedef utility::HeterogeneousMap OptionsMap;

  bool createShape(GenShapePtr & shapeOut, const OptionsMap & shapeOptions) const;
  bool createSphere(GenShapePtr & shapeOut, const OptionsMap & sphereOptions) const;
  bool createBox(GenShapePtr & shapeOut, const OptionsMap & boxOptions) const;
};

}
}

#endif /* GEN_SHAPE_FACTORY_H */

