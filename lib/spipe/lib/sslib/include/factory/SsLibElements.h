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

#include "build_cell/Sphere.h"
#include "common/AtomSpeciesId.h"
#include "io/AtomFormatParser.h"
#include "factory/FactoryFwd.h"
#include "potential/SimplePairPotential.h"
#include "utility/HeterogeneousMap.h"
#include "utility/Range.h"

// DEFINES //////////////////////////////////////////////
namespace sstbx {
namespace common {
class UnitCell;
}
namespace factory {

///////////////////////////////////////////////////////////
// MAP KEYS
///////////////////////////////////////////////////////////

// GENERAL /////////////////////////////////////////////////
extern utility::Key< ::arma::vec3> POSITION;
extern utility::Key<double> RADIUS;
extern utility::Key<double> TOLERANCE;
extern utility::Key<double> CUTOFF;
extern utility::Key<double> MIN;
extern utility::Key<double> MAX;
extern utility::Key<double> MAX_RATIO;

// OPTIMISERS //////////////////////////////////////////////
extern utility::Key<utility::HeterogeneousMap> OPTIMISER;
extern utility::Key<utility::HeterogeneousMap> TPSD;
extern utility::Key<utility::HeterogeneousMap> CASTEP;
extern utility::Key< ::boost::filesystem::path> CASTEP_EXE;
extern utility::Key< ::std::string> CASTEP_SEED;
extern utility::Key<bool> CASTEP_KEEP_INTERMEDIATES;
extern utility::Key<double> PRESSURE;
extern utility::Key<int> MAX_STEPS;

// POTENTIALS //////////////////////////////////////////////
extern utility::Key<utility::HeterogeneousMap> POTENTIAL;
extern utility::Key<utility::HeterogeneousMap> LENNARD_JONES;

extern utility::Key<factory::AtomSpeciesIdVector> SPECIES_LIST;
extern utility::Key< ::arma::mat> LJ_EPSILON;
extern utility::Key< ::arma::mat> LJ_SIGMA;
extern utility::Key< ::arma::mat> LJ_BETA;
extern utility::Key< ::arma::vec> LJ_POWERS;
extern utility::Key<potential::CombiningRule::Value> POT_COMBINING;

// STRUCTURE //////////////////////////////////////
extern utility::Key<utility::HeterogeneousMap> STRUCTURE;
extern utility::Key<AtomsDataEntryList> ATOMS;
extern utility::Key<AtomSpeciesCount> SPECIES;
extern utility::Key< ::std::vector< ::std::string> > ATOMS_FORMAT;

// STRUCTURE BUILDER //////////////////////////////
extern utility::Key<utility::HeterogeneousMap> BUILDER;
extern utility::Key<bool> CLUSTER;
extern utility::Key<utility::HeterogeneousMap> ATOMS_GROUP;
extern utility::Key<utility::HeterogeneousMap> UNIT_CELL_BUILDER;
extern utility::Key< ::std::vector<utility::Range<double> > > UNIT_CELL_BUILDER_ABC;
extern utility::Key<double> UNIT_CELL_BUILDER_VOLUME;
extern utility::Key<utility::HeterogeneousMap> UNIT_CELL_BUILDER_ANGLES;
extern utility::Key<utility::HeterogeneousMap> UNIT_CELL_BUILDER_LENGTHS;
// Shape generators
extern utility::Key<utility::HeterogeneousMap> GEN_SPHERE;
extern utility::Key<utility::HeterogeneousMap> GEN_BOX;
extern utility::Key<double> SHELL_THICKNESS;
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
} // sstbx

#endif /* SSLIB_ELEMENTS_H */

