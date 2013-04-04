/*
 * RandomUnitCellGenerator.h
 *
 *  Created on: Aug 30, 2011
 *      Author: Martin Uhrin
 */

#ifndef RANDOM_UNIT_CELL_GENERATOR_H
#define RANDOM_UNIT_CELL_GENERATOR_H

// INCLUDES ////////////

#include "OptionalTypes.h"
#include "build_cell/IUnitCellGenerator.h"
#include "utility/IndexingEnums.h"

// FORWARD DECLARATIONS ///////


namespace sstbx {
namespace build_cell {

class RandomUnitCellGenerator : public IUnitCellGenerator
{
public:
  typedef ::std::pair<double, bool> ParamValue;

  static const double DEFAULT_MIN_ANGLE;
  static const double DEFAULT_MAX_ANGLE;
  static const double DEFAULT_MIN_LENGTH;
  static const double DEFAULT_MAX_LENGTH;
  static const double DEFAULT_TARGET_VOLUME;
  static const double DEFAULT_VOLUME_DELTA; // Volume can be +/- this value as a percentage of the target
  static const double DEFAULT_MAX_LENGTH_RATIO;

  ParamValue getMin(const size_t param) const;
  ParamValue getMax(const size_t param) const;

  void setMin(const size_t param, const OptionalDouble min = OptionalDouble());
  void setMax(const size_t param, const OptionalDouble max = OptionalDouble());

  void setMinLengths(const OptionalDouble length = OptionalDouble());
  void setMaxLengths(const OptionalDouble length = OptionalDouble());

  void setMinAngles(const OptionalDouble angle = OptionalDouble());
  void setMaxAngles(const OptionalDouble angle = OptionalDouble());

  void setTargetVolume(const OptionalDouble volume = OptionalDouble());
  void setVolumeDelta(const OptionalDouble delta = OptionalDouble());

  void setMaxLengthRatio(const OptionalDouble maxLengthRatio = OptionalDouble());
  ParamValue getMaxLengthRatio() const;

  // From IUnitCellGenerator ////
  virtual GenerationOutcome generateCell(common::UnitCellPtr & cellOut) const;
  virtual GenerationOutcome generateCell(common::UnitCellPtr & cellOut, const StructureContents & contents) const;

  virtual IUnitCellGeneratorPtr clone() const;
  // End from IUnitCellGenerator //////

private:

  typedef ::std::pair<OptionalDouble, OptionalDouble> MinMax;
  typedef ::std::pair<size_t, size_t> MinMaxIndex;

  inline bool isLength(const size_t param) const { return param <= utility::cell_params_enum::C; }

  double generateParameter(const size_t param) const;

  void generateLengths(double (&latticeParams)[6]) const;

  double generateVolume(const double overrideVolume = 0.0) const;

  MinMaxIndex getMinMaxLengths(const double (&latticeParams)[6]) const;

  bool areParametersValid(const double (&latticeParams)[6]) const;

  /** An array of the optional min/max values of the unit cell lattice parameters. */
  MinMax myParameters[6];

  OptionalDouble  myTargetVolume;
  OptionalDouble  myVolumeDelta;
  OptionalDouble  myMaxLengthRatio;
};

}
}

#endif /* RANDOM_UNIT_CELL_GENERATOR_H */
