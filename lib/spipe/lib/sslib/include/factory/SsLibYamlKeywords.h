/*
 * SsLibYamlKeywords.h
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

#ifndef SSLIB_YAML_KEYWORDS_H
#define SSLIB_YAML_KEYWORDS_H

// INCLUDES //////////////////////////////////
#include "SSLib.h"

#ifdef SSLIB_USE_YAML

#include <string>

// NAMESPACES ////////////////////////////////

namespace sstbx {
namespace factory {
namespace sslib_yaml_keywords {

/** The id type used by elements in the yaml document */
typedef ::std::string     KwTyp;

/**
/* A list of the keywords used in the yaml sslib input files
/**/
// General
static const KwTyp ATOMS    = "atoms";
static const KwTyp DEFAULT  = "default";
static const KwTyp MIN      = "min";
static const KwTyp MAX      = "max";
static const KwTyp POS      = "pos";
static const KwTyp RADIUS   = "radius";
static const KwTyp SPEC     = "spec";
static const KwTyp TYPE     = "type";
static const KwTyp VALUE    = "value";
static const KwTyp VOL      = "vol";


// Structure description ///
static const KwTyp STRUCTURE                      = "structure";
static const KwTyp STRUCTURES                     = "structures";
static const KwTyp STRUCTURE__ATOMS_FORMAT        = "atomsFormat";
static const KwTyp STRUCTURE__ATOMS               = ATOMS;
static const KwTyp STRUCTURE__ATOMS__INFO         = "info";
static const KwTyp STRUCTURE__ATOMS__SPEC         = SPEC;
static const KwTyp STRUCTURE__ATOMS__POS          = POS;
static const KwTyp STRUCTURE__ATOMS__RADIUS       = RADIUS;
static const KwTyp STRUCTURE__CELL                = "cell";
static const KwTyp STRUCTURE__CELL__ABC           = "abc";
static const KwTyp STRUCTURE__CELL__VOL           = VOL;
static const KwTyp STRUCTURE__NAME                = "name";
static const KwTyp STRUCTURE__PROPERTIES          = "properties";

// Random structure ///////////////
static const KwTyp RANDOM_STRUCTURE                       = "randomStructure";
static const KwTyp RANDOM_STRUCTURE__ATOMS_RADII          = "atomsRadii";
static const KwTyp RANDOM_STRUCTURE__ATOMS                = ATOMS;
static const KwTyp RANDOM_STRUCTURE__ATOMS__SPEC          = SPEC;
static const KwTyp RANDOM_STRUCTURE__ATOMS__RADIUS        = RADIUS;
static const KwTyp RANDOM_STRUCTURE__ATOMS__POS           = POS;
static const KwTyp RANDOM_STRUCTURE__ATOMS__GEN_SPHERE    = "genSphere";
static const KwTyp RANDOM_STRUCTURE__ATOMS__GROUP         = "group";

// Unit cell ////////////////////////////////
static const KwTyp CELL                       = "cell";
static const KwTyp CELL__ABC                  = "abc";

// Random unit cell /////////////////////////
static const KwTyp RANDOM_CELL                      = "randomCell";
static const KwTyp RANDOM_CELL__ABC                 = CELL__ABC;
static const KwTyp RANDOM_CELL__VOL                 = VOL;
static const KwTyp RANDOM_CELL__VOL_DELTA           = "volDelta";
static const KwTyp RANDOM_CELL__ANGLES              = "angles";
static const KwTyp RANDOM_CELL__ANGLES__MIN         = MIN;
static const KwTyp RANDOM_CELL__ANGLES__MAX         = MAX;
static const KwTyp RANDOM_CELL__LENGTHS             = "lengths";
static const KwTyp RANDOM_CELL__LENGTHS__MIN        = MIN;
static const KwTyp RANDOM_CELL__LENGTHS__MAX        = MAX;
static const KwTyp RANDOM_CELL__LENGTHS__MAX_RATIO  = "maxRatio";

static const KwTyp POTENTIAL                        = "potential";
static const KwTyp POTENTIAL__TYPE                  = TYPE;
static const KwTyp POTENTIAL__TYPE___LENNARD_JONES  = "lennardJones";
static const KwTyp POTENTIAL__LENNARD_JONES__CUT    = "cut";
static const KwTyp POTENTIAL__LENNARD_JONES__EPS    = "eps";
static const KwTyp POTENTIAL__LENNARD_JONES__SIG    = "sig";
static const KwTyp POTENTIAL__LENNARD_JONES__BETA   = "beta";
static const KwTyp POTENTIAL__LENNARD_JONES__POW    = "pow";
static const KwTyp POTENTIAL__LENNARD_JONES__SPECIES= "species";
static const KwTyp POTENTIAL__LENNARD_JONES__COMB   = "comb";

static const KwTyp OPTIMISER                  = "optimiser";
static const KwTyp OPTIMISER__TYPE            = TYPE;
static const KwTyp OPTIMISER__TYPE___TPSD     = "tpsd";
static const KwTyp OPTIMISER__TPSD__TOL       = "tol";
static const KwTyp OPTIMISER__POTENTIAL       = "potential";

static const KwTyp OPTIMISATION_SETTINGS__PRESSURE = "pressure";

static const KwTyp STR_SET                = "strSet";

static const KwTyp STR_COMPARATOR                     = "strComparator";
static const KwTyp STR_COMPARATOR__TYPE___SORTED_DIST = "sortedDist";

static const KwTyp STR_WRITER             = "strWriter";
static const KwTyp STR_WRITER__TYPE___RES = "res";

// SPHERE
static const KwTyp SPHERE = "sphere";
static const KwTyp SPHERE__POS    = POS;
static const KwTyp SPHERE__RADIUS = RADIUS;
static const KwTyp SPHERE__VOL    = VOL;

}
}
}

#endif /* SSLIB_YAML_KEYWORDS_H */

#endif /* SSLIB_USE_YAML */
