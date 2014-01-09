/*
 * SsLibElements.h
 *
 *
 *  Created on: May 29, 2012
 *      Author: Martin Uhrin
 */

#ifndef SSLIB_ELEMENTS_H
#define SSLIB_ELEMENTS_H

// INCLUDES /////////////////////////////////////////////
#include <string>

#include <boost/filesystem/path.hpp>
#include <boost/variant.hpp>

#include <armadillo>

#include "spl/build_cell/Sphere.h"
#include "spl/common/AtomSpeciesId.h"
#include "spl/io/AtomFormatParser.h"
#include "spl/factory/FactoryFwd.h"
#include "spl/potential/LennardJones.h"
#include "spl/utility/HeterogeneousMap.h"
#include "spl/utility/Range.h"

// DEFINES //////////////////////////////////////////////
namespace spl {
namespace common {
class UnitCell;
}
namespace factory {

///////////////////////////////////////////////////////////
// MAP KEYS
///////////////////////////////////////////////////////////

// GENERAL /////////////////////////////////////////////////
extern utility::Key< ::std::string> LABEL;
extern utility::Key< ::arma::vec3> POSITION;
extern utility::Key< ::arma::vec4> ROT_AXIS_ANGLE;
extern utility::Key<double> RADIUS;
extern utility::Key<double> TOLERANCE;
extern utility::Key<double> CUTOFF;
extern utility::Key<double> MIN;
extern utility::Key<double> MAX;
extern utility::Key<double> MAX_RATIO;
extern utility::Key<int> NUM;

// OPTIMISERS //////////////////////////////////////////////
extern utility::Key<utility::HeterogeneousMap> OPTIMISER;
extern utility::Key<bool> WRITE_SUMMARY;
extern utility::Key<utility::HeterogeneousMap> TPSD;
extern utility::Key<utility::HeterogeneousMap> CASTEP;
extern utility::Key< ::std::string> CASTEP_EXE;
extern utility::Key< ::std::string> CASTEP_SEED;
extern utility::Key<bool> CASTEP_KEEP_INTERMEDIATES;
extern utility::Key<int> CASTEP_NUM_SELF_CONSISTENT;
extern utility::Key<int> CASTEP_NUM_ROUGH_STEPS;
extern utility::Key<double> PRESSURE;
extern utility::Key<int> MAX_ITER;
extern utility::Key<double> ENERGY_TOL;
extern utility::Key<double> FORCE_TOL;
extern utility::Key<double> STRESS_TOL;

// POTENTIALS //////////////////////////////////////////////
extern utility::Key<utility::HeterogeneousMap> POTENTIAL;
extern utility::Key<utility::HeterogeneousMap> LENNARD_JONES;

extern utility::Key< ::arma::mat> LJ_EPSILON;
extern utility::Key< ::arma::mat> LJ_SIGMA;
extern utility::Key< ::arma::mat> LJ_BETA;
extern utility::Key< ::arma::vec2> LJ_POWERS;
extern utility::Key<potential::CombiningRule::Value> POT_COMBINING;

// STRUCTURE //////////////////////////////////////
extern utility::Key<utility::HeterogeneousMap> STRUCTURE;
extern utility::Key<AtomSpeciesCount> SPECIES;
extern utility::Key< ::std::vector< ::std::string> > ATOMS_FORMAT;

// STRUCTURE BUILDER //////////////////////////////
extern utility::Key<utility::HeterogeneousMap> BUILDER;
extern utility::Key<bool> CLUSTER;
extern utility::Key<double> ATOMS_OVERLAP;
extern utility::Key<utility::HeterogeneousMap> ATOMS_GROUP;
extern utility::Key<utility::HeterogeneousMap> UNIT_CELL_BUILDER;
extern utility::Key< ::std::vector<utility::Range<double> > > UNIT_CELL_BUILDER_ABC;
extern utility::Key<double> UNIT_CELL_BUILDER_VOLUME;
extern utility::Key<double> UNIT_CELL_BUILDER_VOLUME_DELTA;
extern utility::Key<double> UNIT_CELL_BUILDER_MULTIPLIER;
extern utility::Key<utility::HeterogeneousMap> UNIT_CELL_BUILDER_ANGLES;
extern utility::Key<utility::HeterogeneousMap> UNIT_CELL_BUILDER_LENGTHS;
extern utility::Key< ::std::string> POSITION_STRING;
extern utility::Key< ::std::string> ROTATION_STRING;
extern utility::Key< ::std::map< ::std::string, double> > PAIR_DISTANCES;
// Shape generators
extern utility::Key<utility::HeterogeneousMap> GEN_BOX;
extern utility::Key<utility::HeterogeneousMap> GEN_CYLINDER;
extern utility::Key<utility::HeterogeneousMap> GEN_SHAPE;
extern utility::Key<utility::HeterogeneousMap> GEN_SPHERE;
extern utility::Key<double> SHELL_THICKNESS;
extern utility::Key<bool> SHELL_CAPPED;
extern utility::Key<double> WIDTH;
extern utility::Key<double> HEIGHT;
extern utility::Key<double> DEPTH;
// Symmetry
extern utility::Key<utility::HeterogeneousMap> SYMMETRY;
extern utility::Key<MinMax> SYM_OPS;
extern utility::Key< ::std::string> POINT_GROUP;

// STRUCTURE COMPARATORS //////////////////////////
extern utility::Key<utility::HeterogeneousMap> COMPARATOR;
extern utility::Key<utility::HeterogeneousMap> SORTED_DISTANCE;
extern utility::Key<bool> SORTED_DISTANCE__VOLUME_AGNOSTIC;
extern utility::Key<bool> SORTED_DISTANCE__USE_PRIMITIVE;

// UNIT CELL //////////////////////////////////////
extern utility::Key<common::UnitCell> UNIT_CELL;
extern utility::Key< ::std::vector<double> > ABC;


} // factory
} // spl

#endif /* SSLIB_ELEMENTS_H */

