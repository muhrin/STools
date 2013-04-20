/*
 * GenShapeFactory.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "factory/GenShapeFactory.h"


// Local includes
#include "build_cell/GenBox.h"
#include "build_cell/GenSphere.h"
#include "factory/SsLibElements.h"
#include "math/Matrix.h"

// NAMESPACES ////////////////////////////////

namespace sstbx {
namespace factory {

// namespace aliases
namespace ssbc  = ::sstbx::build_cell;
namespace ssf   = ::sstbx::factory;
namespace ssu   = ::sstbx::utility;

bool GenShapeFactory::createShape(GenShapePtr & shapeOut, const OptionsMap & shapeOptions) const
{
  const OptionsMap * const sphereOptions = shapeOptions.find(GEN_SPHERE);
  if(sphereOptions)
    return createSphere(shapeOut, *sphereOptions);

  const OptionsMap * const boxOptions = shapeOptions.find(GEN_BOX);
  if(boxOptions)
    return createBox(shapeOut, *boxOptions);

  return false;
}

bool GenShapeFactory::createSphere(GenShapePtr & shapeOut, const OptionsMap & sphereOptions) const
{
  const double * const radius = sphereOptions.find(ssf::RADIUS);
  const ::arma::vec3 * const pos = sphereOptions.find(ssf::POSITION);
  const double * const thickness = sphereOptions.find(ssf::SHELL_THICKNESS);

  if(!radius)
    return false;

  UniquePtr<ssbc::GenSphere>::Type sphere(new ssbc::GenSphere(*radius));
  if(pos)
    sphere->setPosition(*pos);

  if(thickness)
    sphere->setShellThickness(*thickness);
  
  shapeOut = sphere;
  return true;
}

bool GenShapeFactory::createBox(GenShapePtr & shapeOut, const OptionsMap & boxOptions) const
{
  const double * const width = boxOptions.find(ssf::WIDTH);
  const double * const height = boxOptions.find(ssf::HEIGHT);
  const double * const depth = boxOptions.find(ssf::DEPTH);
  const ::arma::vec3 * const pos = boxOptions.find(ssf::POSITION);
  const ::arma::vec4 * const rot = boxOptions.find(ssf::ROT_AXIS_ANGLE);
  const double * const thickness = boxOptions.find(ssf::SHELL_THICKNESS);

  ::arma::mat44 transform;
  transform.eye();
  if(pos)
    math::setTranslation(transform, *pos);
  if(rot)
    math::setRotation(transform, *rot);

  if(!width || !height || !depth)
    return false;

  UniquePtr<ssbc::GenBox>::Type box(new ssbc::GenBox(*width, *height, *depth, transform));

  if(thickness)
    box->setShellThickness(*thickness);
  
  shapeOut = box;
  return true;
}

}
}


