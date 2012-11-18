/*
 * YamlKeywords.h
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

#ifndef YAML_KEYWORDS_H
#define YAML_KEYWORDS_H

// INCLUDES //////////////////////////////////

#include <string>

// NAMESPACES ////////////////////////////////

namespace spipe
{
namespace common
{
namespace yaml_keywords
{

/** The id type used by elements in the yaml document */
typedef ::std::string     IdTyp;

/**
/* A list of the keywords used in the yaml pipeline input files
/**/
static const IdTyp ID                        = "id";
static const IdTyp TYPE                      = "type";
static const IdTyp VALUE                     = "value";
// Pipeline
static const IdTyp PIPELINE                  = "pipeline";
static const IdTyp PIPELINE__SEQ             = "seq";
static const IdTyp PIPELINE__SINGLE_THREADED = "singleThreaded";
// Blocks
static const IdTyp BLOCKS                         = "blocks";
static const IdTyp BLOCKS__BARRIER                = "barrier";
static const IdTyp BLOCKS__DROP_DUPLICATES        = "dropDuplicates";
static const IdTyp BLOCKS__RANDOM_CRYSTAL         = "randomCrystal";
static const IdTyp BLOCKS__RANDOM_CRYSTAL__N_STRS = "nStrs";

static const IdTyp BLOCKS__NIGGLI                 = "niggli";

static const IdTyp BLOCKS__GEOM_OPTIMISE              = "geomOptimise";
static const IdTyp BLOCKS__GEOM_OPTIMISE__OPTIMISER   = "optimiser";

static const IdTyp BLOCKS__WRITE_STRS             = "writeStrs";

}
}
}


#endif /* YAML_KEYWORDS_H */
