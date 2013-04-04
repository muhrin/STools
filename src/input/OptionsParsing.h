/*
 * OptionsParsing.h
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

#ifndef OPTIONS_PARSING_H
#define OPTIONS_PARSING_H

// INCLUDES //////////////////////////////////
#include <string>

// NAMESPACES ////////////////////////////////
namespace YAML {
class Node;
}

namespace stools {
namespace input {

int parseYaml(YAML::Node & nodeOut, const ::std::string & inputFile); 

}
}

#endif /* OPTIONS_PARSING_H */
