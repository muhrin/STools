/*
 * Initialisation.h
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

#ifndef INITIALISTAION_H
#define INITIALISATION_H

// INCLUDES //////////////////////////////////
#include <string>
#include <vector>

// NAMESPACES ////////////////////////////////
namespace YAML {
class Node;
}

namespace spipe {
namespace utility {

int
parseYaml(YAML::Node & nodeOut, const ::std::string & inputFile);
bool
insertScalarValues(YAML::Node & node,
    const ::std::vector< ::std::string> & scalarValues);

void
seedRandomNumberGenerator(
    const ::std::string & seed);

}
}

#endif /* INITIALISATION_H */
