/*
 * SsLibYamlKeywords.h
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

#ifndef SSLIB_YAML_KEYWORDS_H
#define SSLIB_YAML_KEYWORDS_H

// INCLUDES //////////////////////////////////

#include <string>

// NAMESPACES ////////////////////////////////

namespace spipe
{
namespace common
{
namespace sslib_yaml_keywords
{

/** The id type used by elements in the yaml document */
typedef ::std::string     KwTyp;

/**
/* A list of the keywords used in the yaml sslib input files
/**/
// General
static const KwTyp TYPE                  = "type";
static const KwTyp VALUE                 = "value";
static const KwTyp DEFAULT               = "default";

// Structure description ///
static const KwTyp STR_DESC              = "strDesc";
static const KwTyp STR_DESC__ATOMS       = "atoms";
static const KwTyp STR_DESC__ATOMS__SPEC = "spec";
static const KwTyp STR_DESC__ATOMS__N    = "n";
static const KwTyp STR_DESC__CONSTRAINTS = "constraints";
static const KwTyp STR_DESC__CONSTRAINTS__TYPE___MINSEP = "minsep";

static const KwTyp STR_GENERATOR                    = "strGenerator";
static const KwTyp STR_GENERATOR__TYPE___DEFAULT    = DEFAULT;

static const KwTyp CELL_DESC                  = "cellDesc";
static const KwTyp CELL_DESC__PARAMS          = "params";
static const KwTyp CELL_DESC__VOL             = "vol";
static const KwTyp CELL_DESC__VOL_DELTA       = "volDelta";
static const KwTyp CELL_DESC__MIN_ANGLE       = "minAngle";
static const KwTyp CELL_DESC__MAX_ANGLE       = "maxAngle";

static const KwTyp CELL_GENERATOR                   = "cellGenerator";
static const KwTyp CELL_GENERATOR__TYPE___DEFAULT   = DEFAULT;

static const KwTyp OPTIMISER__TYPE___TPSD   = "tpsd";

static const KwTyp POTENTIAL                    = "potential";
static const KwTyp POTENTIAL__TYPE___PAIR_POT   = "pairPot";
static const KwTyp POTENTIAL__PAIR_POT__E       = "e";
static const KwTyp POTENTIAL__PAIR_POT__S       = "s";
static const KwTyp POTENTIAL__PAIR_POT__CUT     = "cut";
static const KwTyp POTENTIAL__PAIR_POT__B       = "b";
static const KwTyp POTENTIAL__PAIR_POT__POW     = "pow";
static const KwTyp POTENTIAL__PAIR_POT__COMB    = "comb";

static const KwTyp STR_SET                = "strSet";

static const KwTyp STR_COMPARATOR                     = "strComparator";
static const KwTyp STR_COMPARATOR__TYPE___SORTED_DIST = "sortedDist";

static const KwTyp STR_WRITER             = "strWriter";
static const KwTyp STR_WRITER__TYPE___RES = "res";


}
}
}

#endif /* SSLIB_YAML_KEYWORDS_H */

