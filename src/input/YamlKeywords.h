/*
 * YamlKeywords.h
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

#ifndef STOOLS_YAML_KEYWORDS_H
#define STOOLS_YAML_KEYWORDS_H

// INCLUDES //////////////////////////////////
#include <string>

// NAMESPACES ////////////////////////////////

namespace stools {
namespace input {
namespace yaml_kw {

/** The id type used by elements in the yaml document */
typedef ::std::string Keyword;


static const Keyword STOICHIOMETRY_SEARCH = "stoichiometrySearch";
static const Keyword STOICHIOMETRY_SEARCH__MAX_ATOMS = "maxAtoms";
static const Keyword STOICHIOMETRY_SEARCH__SPECIES = "species";

static const Keyword SEED_STRUCTURES = "seedStructures";

}
}
}

#endif /* STOOLS_YAML_KEYWORDS_H */
