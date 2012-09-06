/*
 * RandomUnitCell.cpp
 *
 *  Created on: Nov 10, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////////
#include "build_cell/RandomUnitCell.h"

#include "SSLibAssert.h"
#include "common/UnitCell.h"
#include "common/Utils.h"


namespace sstbx {
namespace build_cell {

const double RandomUnitCell::DEFAULT_MIN_ANGLE = 30.0;
const double RandomUnitCell::DEFAULT_MAX_ANGLE = 140.0;
const double RandomUnitCell::DEFAULT_MIN_LENGTH = 0.25;
const double RandomUnitCell::DEFAULT_MAX_LENGTH = 2.0;
const double RandomUnitCell::DEFAULT_TARGET_VOLUME = 50.0;
const double RandomUnitCell::DEFAULT_VOLUME_DELTA = 0.25;

RandomUnitCell::ParamValue RandomUnitCell::getMin(const size_t param) const
{
  SSLIB_ASSERT(param < 6);

  ParamValue returnVal;
  if(myParameters[param].first)
  {
    returnVal.first = *myParameters[param].first;
    returnVal.second = true;
  }
  else
  {
    if(param < 3)
    {
      returnVal.first = DEFAULT_MIN_LENGTH;
    }
    else
    {
      returnVal.first = DEFAULT_MIN_ANGLE;
    }
    returnVal.second = false;
  }

  return returnVal;
}

RandomUnitCell::ParamValue RandomUnitCell::getMax(const size_t param) const
{
  SSLIB_ASSERT(param < 6);

  ParamValue returnVal;
  if(myParameters[param].second)
  {
    returnVal.first = *myParameters[param].second;
    returnVal.second = true;
  }
  else
  {
    if(param < 3)
    {
      returnVal.first = DEFAULT_MAX_LENGTH;
    }
    else
    {
      returnVal.first = DEFAULT_MAX_ANGLE;
    }
    returnVal.second = false;
  }

  return returnVal;
}

void RandomUnitCell::setMin(const size_t param, const OptionalDouble min)
{
  SSLIB_ASSERT(param < 6);

  myParameters[param].first = min;
}

void RandomUnitCell::setMax(const size_t param, const OptionalDouble max)
{
  SSLIB_ASSERT(param < 6);

  myParameters[param].second = max;
}

void RandomUnitCell::setMinLengths(const OptionalDouble length)
{
  for(size_t i = 0; i < 3; ++i)
  {
    myParameters[i].first = length;
  }
}

void RandomUnitCell::setMaxLengths(const OptionalDouble length)
{
  for(size_t i = 0; i < 3; ++i)
  {
    myParameters[i].second = length;
  }
}

void RandomUnitCell::setMinAngles(const OptionalDouble angle)
{
  for(size_t i = 3; i < 6; ++i)
  {
    myParameters[i].first = angle;
  }
}

void RandomUnitCell::setMaxAngles(const OptionalDouble angle)
{
  for(size_t i = 3; i < 6; ++i)
  {
    myParameters[i].second = angle;
  }
}

void RandomUnitCell::setTargetVolume(const OptionalDouble volume)
{
  myTargetVolume = volume;
}

void RandomUnitCell::setVolumeDelta(const OptionalDouble delta)
{
  myVolumeDelta = delta;
}

::boost::shared_ptr<common::UnitCell> RandomUnitCell::generateCell(const RandomUnitCell::OptionalStructureInfo structureInfo) const
{
  double params[6];

  size_t i;
  // Generate the lengths
  for(i = 0; i < 3; ++i)
  {
    params[i] = generateParameter(i);
  }

  bool anglesValid = false;
  for(size_t iters = 0; iters < 1000 && !anglesValid; ++iters)
  {
    for(i = 3; i < 6; ++i)
    {
      params[i] = generateParameter(i);
    }
    anglesValid = areParametersValid(params);
  }

  // If the angles still aren't valid then use the
  // minimum values for all of them
  if(!anglesValid)
  {
    for(i = 3; i < 6; ++i)
    {
      params[i] = getMin(i).first;
    }
  }

  ::boost::shared_ptr<common::UnitCell> cell(new common::UnitCell(params));

  if(!myTargetVolume && structureInfo)
  {
    cell->setVolume(2.0 * structureInfo->atomsVolume); 
  }
  else
  {
    cell->setVolume(generateVolume());
  }

  return cell;
}

double RandomUnitCell::generateParameter(const size_t param) const
{
  SSLIB_ASSERT(param < 6);

  if(isLength(param))
  {
    const double min = myParameters[param].first ? *myParameters[param].first : DEFAULT_MIN_LENGTH;
    const double max = myParameters[param].second ? *myParameters[param].second : DEFAULT_MAX_LENGTH;

    return common::randDouble(min, max);
  }
  else
  {
    const double min = myParameters[param].first ? *myParameters[param].first : DEFAULT_MIN_ANGLE;
    const double max = myParameters[param].second ? *myParameters[param].second : DEFAULT_MAX_ANGLE;

    return common::randDouble(min, max);    
  }
}

double RandomUnitCell::generateVolume(const double overrideVolume) const
{
  double target = overrideVolume;
  if(target == 0.0)
  {
    target = myTargetVolume ? *myTargetVolume : DEFAULT_TARGET_VOLUME;
  }

  const double delta = myVolumeDelta ? *myVolumeDelta : DEFAULT_VOLUME_DELTA;

  return common::randDouble(target * (1.0 - delta), target * (1 + delta));
}

bool RandomUnitCell::areParametersValid(const double (&params)[6]) const
{
  double anglesSum = params[ALPHA] + params[BETA] + params[GAMMA];

  if(anglesSum >= 360.0) return false;
  if(abs(params[ALPHA]-params[BETA]) > params[GAMMA]) return false;
  if(abs(params[BETA]-params[GAMMA]) > params[ALPHA]) return false;
  if(abs(params[GAMMA]-params[ALPHA]) > params[BETA]) return false;

  return true;
}

}
}
