/*
 * SsLibElements.cpp
 *
 *  Created on: May 29, 2012
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "factory/SsLibElements.h"

// NAMESPACES ////////////////////////////////

namespace sstbx {
namespace factory {

// GENERAL /////////////////////////////////////////////////
utility::Key< ::arma::vec3> POSITION;
utility::Key<double> RADIUS;
utility::Key<double> TOLERANCE;
utility::Key<double> CUTOFF;
utility::Key<double> MIN;
utility::Key<double> MAX;
utility::Key<double> MAX_RATIO;

// OPTIMISERS //////////////////////////////////////////////
utility::Key<utility::HeterogeneousMap> OPTIMISER;
utility::Key<utility::HeterogeneousMap> TPSD;
utility::Key<utility::HeterogeneousMap> CASTEP;
utility::Key< ::std::string> CASTEP_EXE;
utility::Key< ::std::string> CASTEP_SEED;
utility::Key<bool> CASTEP_KEEP_INTERMEDIATES;
utility::Key<int> CASTEP_NUM_SELF_CONSISTENT;
utility::Key<int> CASTEP_NUM_ROUGH_STEPS;
utility::Key<double> PRESSURE;
utility::Key<int> MAX_STEPS;

// POTENTIALS //////////////////////////////////////////////
utility::Key<utility::HeterogeneousMap> POTENTIAL;
utility::Key<utility::HeterogeneousMap> LENNARD_JONES;

utility::Key<factory::AtomSpeciesIdVector> SPECIES_LIST;
utility::Key< ::arma::mat> LJ_EPSILON;
utility::Key< ::arma::mat> LJ_SIGMA;
utility::Key< ::arma::mat> LJ_BETA;
utility::Key< ::arma::vec> LJ_POWERS;
utility::Key<potential::CombiningRule::Value> POT_COMBINING;

// STRUCTURE //////////////////////////////////////
utility::Key<utility::HeterogeneousMap> STRUCTURE;
utility::Key<AtomsDataEntryList> ATOMS;
utility::Key<AtomSpeciesCount> SPECIES;
utility::Key< ::std::vector< ::std::string> > ATOMS_FORMAT;

// STRUCTURE BUILDER //////////////////////////////
utility::Key<utility::HeterogeneousMap> BUILDER;
utility::Key<bool> CLUSTER;
utility::Key<utility::HeterogeneousMap> ATOMS_GROUP;
utility::Key<utility::HeterogeneousMap> UNIT_CELL_BUILDER;
utility::Key< ::std::vector<utility::Range<double> > > UNIT_CELL_BUILDER_ABC;
utility::Key<double> UNIT_CELL_BUILDER_VOLUME;
utility::Key<utility::HeterogeneousMap> UNIT_CELL_BUILDER_ANGLES;
utility::Key<utility::HeterogeneousMap> UNIT_CELL_BUILDER_LENGTHS;
// Shape generators
utility::Key<utility::HeterogeneousMap> GEN_SPHERE;
utility::Key<utility::HeterogeneousMap> GEN_BOX;
utility::Key<double> SHELL_THICKNESS;
utility::Key<double> WIDTH;
utility::Key<double> HEIGHT;
utility::Key<double> DEPTH;
// Symmetry
utility::Key<utility::HeterogeneousMap> SYMMETRY;
utility::Key<MinMax> SYM_OPS;
utility::Key< ::std::string> POINT_GROUP;

// STRUCTURE COMPARATORS //////////////////////////
utility::Key<utility::HeterogeneousMap> COMPARATOR;
utility::Key<utility::HeterogeneousMap> SORTED_DISTANCE;
utility::Key<bool> SORTED_DISTANCE__VOLUME_AGNOSTIC;
utility::Key<bool> SORTED_DISTANCE__USE_PRIMITIVE;

// UNIT CELL //////////////////////////////////////
utility::Key<common::UnitCell> UNIT_CELL;
utility::Key< ::std::vector<double> > ABC;

}
}


