/*
 * SsLibYamlSchema.h
 *
 *
 *  Created on: May 29, 2012
 *      Author: Martin Uhrin
 */

#ifndef SSLIB_YAML_SCHEMA_H
#define SSLIB_YAML_SCHEMA_H

// INCLUDES /////////////////////////////////////////////
#include "spl/SSLib.h"

#ifdef SSLIB_USE_YAML

#include "spl/factory/SsLibElements.h"
#include "spl/potential/TpsdGeomOptimiser.h"
#include "spl/utility/SortedDistanceComparator.h"
#include "spl/yaml/Transcode.h"
#include "spl/yaml_schema/YamlSchema.h"

// DEFINES //////////////////////////////////////////////

namespace spl {
namespace factory {

// TYPEDEFS //////////////////////////////////////
typedef yaml_schema::SchemaHeteroMap HeteroMap;

///////////////////////////////////////////////////////////
// CUSTOM MAPS
///////////////////////////////////////////////////////////
typedef yaml_schema::SchemaList< yaml_schema::SchemaScalar< double> > SchemaDoubleList;

// POTENTIALS ////////////////////////////////////////////////

struct LennardJones : public HeteroMap
{
  LennardJones()
  {
    addEntry("spec", SPECIES_LIST,
        new yaml_schema::SchemaWrapper< yaml::VectorAsString< ::std::string> >)->required();
    addEntry("eps", LJ_EPSILON,
        new yaml_schema::SchemaWrapper< yaml::ArmaTriangularMat>())->required();
    addEntry("sig", LJ_SIGMA,
        new yaml_schema::SchemaWrapper< yaml::ArmaTriangularMat>())->required();
    addEntry("beta", LJ_BETA,
        new yaml_schema::SchemaWrapper< yaml::ArmaTriangularMat>())->required();

    ::arma::vec2 powers;
    powers(0) = 12;
    powers(1) = 6;
    addScalarEntry("pow", LJ_POWERS)->element()->defaultValue(powers);

    addEntry("combining", POT_COMBINING,
        (new yaml_schema::SchemaScalar< potential::CombiningRule::Value>)->defaultValue(
            potential::CombiningRule::NONE));

    addScalarEntry("cut", CUTOFF)->element()->defaultValue(2.5);
  }
};

struct Potential : public yaml_schema::SchemaHeteroMap
{
  Potential()
  {
    addEntry("lennardJones", LENNARD_JONES, new LennardJones());
  }
};

// OPTIMISERS ////////////////////////////////////////////////
struct OptimisationSettings : public HeteroMap
{
  OptimisationSettings()
  {
    addScalarEntry("maxIter", MAX_ITER);
    addScalarEntry("energyTol", ENERGY_TOL);
    addScalarEntry("forceTol", FORCE_TOL);
    addScalarEntry("stressTol", STRESS_TOL);
    addScalarEntry("pressure", PRESSURE);
  }
};

struct Tpsd : OptimisationSettings
{
  Tpsd()
  {
    addScalarEntry("maxIter", MAX_ITER)->element()->defaultValue(
        potential::TpsdGeomOptimiser::DEFAULT_MAX_ITERATIONS);
    addScalarEntry("energyTol", ENERGY_TOL)->element()->defaultValue(
        potential::TpsdGeomOptimiser::DEFAULT_ENERGY_TOLERANCE);
    addScalarEntry("forceTol", FORCE_TOL)->element()->defaultValue(
        potential::TpsdGeomOptimiser::DEFAULT_FORCE_TOLERANCE);
    addScalarEntry("stressTol", STRESS_TOL)->element()->defaultValue(
        potential::TpsdGeomOptimiser::DEFAULT_STRESS_TOLERANCE);
    addScalarEntry("pressure", PRESSURE)->element()->defaultValue(0.0);
    addEntry("potential", POTENTIAL, new Potential())->required();
  }
};

struct Castep : public HeteroMap
{
  Castep()
  {
    addScalarEntry("exe", CASTEP_EXE)->element()->defaultValue("castep")->required();
    addScalarEntry("keep", CASTEP_KEEP_INTERMEDIATES)->element()->defaultValue(
        false);
    addScalarEntry("seed", CASTEP_SEED)->required();
    addScalarEntry("roughSteps", CASTEP_NUM_ROUGH_STEPS)->element()->defaultValue(
        0);
    addScalarEntry("consistent", CASTEP_NUM_SELF_CONSISTENT)->element()->defaultValue(
        2);
  }
};

struct Optimiser : public yaml_schema::SchemaHeteroMap
{
  Optimiser()
  {
    addEntry("tpsd", TPSD, new Tpsd());
    addEntry("castep", CASTEP, new Castep());
  }
};

// STRUCTURE //////////////////////////////////////
struct AtomsDataMap : public HeteroMap
{
  typedef utility::HeterogeneousMap BindingType;
  AtomsDataMap()
  {
    addScalarEntry("spec", SPECIES)->required();
  }
};

typedef yaml_schema::SchemaList< yaml_schema::SchemaScalar< ::std::string> > AtomsDataListSchema;
typedef yaml_schema::SchemaListMap< AtomsDataListSchema, AtomsDataMap> AtomsDataSchema;
typedef yaml_schema::SchemaList< AtomsDataSchema> AtomsListSchema;

struct Structure : public yaml_schema::SchemaHeteroMap
{
  Structure()
  {
    addEntry("atoms", ATOMS, new AtomsListSchema());
  }
};

// STRUCTURE BUILDER //////////////////////////////

namespace builder {

struct GenSphere : HeteroMap
{
  typedef utility::HeterogeneousMap BindingType;
  GenSphere()
  {
    addScalarEntry("radius", RADIUS)->required();
    addScalarEntry("pos", POSITION)->element()->defaultValue(
        ::arma::zeros< ::arma::vec>(3));
    addScalarEntry("rot", ROT_AXIS_ANGLE)->element()->defaultValue(
        ::arma::zeros< ::arma::vec>(4));
    addScalarEntry("shell", SHELL_THICKNESS);
  }
  ;
};

struct GenBox : HeteroMap
{
  typedef utility::HeterogeneousMap BindingType;
  GenBox()
  {
    addScalarEntry("pos", POSITION)->element()->defaultValue(
        ::arma::zeros< ::arma::vec>(3));
    addScalarEntry("rot", ROT_AXIS_ANGLE)->element()->defaultValue(
        ::arma::zeros< ::arma::vec>(4));
    addScalarEntry("shell", SHELL_THICKNESS);
    addScalarEntry("width", WIDTH)->required();
    addScalarEntry("height", HEIGHT)->required();
    addScalarEntry("depth", DEPTH)->required();
  }
  ;
};

struct MinMaxRatio : HeteroMap
{
  typedef utility::HeterogeneousMap BindingType;
  MinMaxRatio()
  {
    addScalarEntry("min", MIN);
    addScalarEntry("max", MAX);
    addScalarEntry("maxRatio", MAX_RATIO);
  }
};

struct UnitCellBuilder : public yaml_schema::SchemaHeteroMap
{
  typedef utility::HeterogeneousMap BindingType;
  UnitCellBuilder()
  {
    typedef utility::Range< double> DoubleRange;
    addEntry("abc", UNIT_CELL_BUILDER_ABC,
        new yaml_schema::SchemaWrapper< yaml::VectorAsString< DoubleRange> >());
    addScalarEntry("vol", UNIT_CELL_BUILDER_VOLUME);
    addScalarEntry("delta", UNIT_CELL_BUILDER_VOLUME_DELTA);
    addScalarEntry("mul", UNIT_CELL_BUILDER_MULTIPLIER);
    addEntry("lengths", UNIT_CELL_BUILDER_LENGTHS, new MinMaxRatio());
    addEntry("angles", UNIT_CELL_BUILDER_ANGLES, new MinMaxRatio());

  }
};

struct SimpleAtomsDataMap : HeteroMap
{
  typedef utility::HeterogeneousMap BindingType;
  SimpleAtomsDataMap()
  {
    addScalarEntry("spec", SPECIES);
    addScalarEntry("radius", RADIUS);
    addScalarEntry("pos", POSITION);
    addScalarEntry("label", LABEL);
  }
};

typedef yaml_schema::SchemaListMap<
    yaml_schema::SchemaList< yaml_schema::SchemaScalar< ::std::string> >,
    SimpleAtomsDataMap> SimpleAtomsListEntrySchema;

typedef yaml_schema::SchemaList< SimpleAtomsListEntrySchema> SimpleAtomsListSchema;

struct AtomsGroup : HeteroMap
{
  typedef utility::HeterogeneousMap BindingType;
  AtomsGroup()
  {
    addEntry("genSphere", GEN_SPHERE, new GenSphere());
    addEntry("genBox", GEN_BOX, new GenBox());
    addEntry("atoms", ATOMS, new SimpleAtomsListSchema())->required();
    addScalarEntry("atomsRadius", RADIUS);
    addScalarEntry("pos", POSITION);
    addScalarEntry("rot", ROT_AXIS_ANGLE);
    addScalarEntry("num", NUM)->element()->defaultValue(1);
    addEntry("pairDistances", PAIR_DISTANCES,
        new yaml_schema::SchemaHomoMap< yaml_schema::SchemaScalar< double> >());
  }
};

struct ExtendedAtomsDataMap : public SimpleAtomsDataMap
{
  typedef utility::HeterogeneousMap BindingType;
  ExtendedAtomsDataMap()
  {
    addEntry("group", ATOMS_GROUP, new AtomsGroup());
  }
};

struct Symmetry : HeteroMap
{
  typedef utility::HeterogeneousMap BindingType;

  Symmetry()
  {
    addScalarEntry("ops", SYM_OPS);
    addScalarEntry("pointGroup", POINT_GROUP);
  }
};

struct Builder : HeteroMap
{
  typedef utility::HeterogeneousMap BindingType;

  typedef yaml_schema::SchemaListMap<
      yaml_schema::SchemaList< yaml_schema::SchemaScalar< ::std::string> >,
      ExtendedAtomsDataMap> ExtendedAtomsListEntrySchema;
  typedef yaml_schema::SchemaList< ExtendedAtomsListEntrySchema> ExtendedAtomsListSchema;

  Builder()
  {
    addEntry("atomsFormat", ATOMS_FORMAT,
        new yaml_schema::SchemaWrapper< yaml::VectorAsString< ::std::string> >());
    addScalarEntry("atomsRadius", RADIUS);
    addScalarEntry("cluster", CLUSTER)->element()->defaultValue(false);
    addEntry("atoms", ATOMS, (new ExtendedAtomsListSchema()));
    addEntry("genSphere", GEN_SPHERE, new GenSphere());
    addEntry("genBox", GEN_BOX, new GenBox());
    addEntry("unitCell", UNIT_CELL_BUILDER, new UnitCellBuilder());
    addEntry("symmetry", SYMMETRY, new Symmetry());
    addEntry("pairDistances", PAIR_DISTANCES,
        new yaml_schema::SchemaHomoMap< yaml_schema::SchemaScalar< double> >());
    addScalarEntry("overlap", ATOMS_OVERLAP);
  }
};

} // namespace builder

// STRUCTURE COMPARATORS //////////////////////////

struct SortedDistance : HeteroMap
{
  SortedDistance()
  {
    addScalarEntry("tol", TOLERANCE)->element()->defaultValue(
        utility::SortedDistanceComparator::DEFAULT_TOLERANCE);
    addScalarEntry("volAgnostic", SORTED_DISTANCE__VOLUME_AGNOSTIC)->element()->defaultValue(
        false);
    addScalarEntry("usePrimitive", SORTED_DISTANCE__USE_PRIMITIVE)->element()->defaultValue(
        false);
  }
};

struct Comparator : HeteroMap
{
  Comparator()
  {
    addEntry("sortedDist", SORTED_DISTANCE, new SortedDistance());

    // Defaults
    utility::HeterogeneousMap comparatorDefault;
    comparatorDefault[SORTED_DISTANCE];
    defaultValue(comparatorDefault);
  }
};

// UNIT CELL //////////////////////////////////////////

struct UnitCell : HeteroMap
{
  UnitCell()
  {
    addEntry("abc", ABC, (new SchemaDoubleList())->length(6)->required());
  }
};

}
}

#endif /* SSLIB_YAML_SCHEMA_H */

#endif /* SSLIB_USE_YAML */
