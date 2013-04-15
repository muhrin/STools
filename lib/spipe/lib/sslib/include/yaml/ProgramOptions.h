/*
 * ProgramOptions.h
 *
 *
 *  Created on: May 29, 2012
 *      Author: Martin Uhrin
 */

#ifndef PROGRAM_OPTIONS_H
#define PROGRAM_OPTIONS_H

// INCLUDES /////////////////////////////////////////////
#include "SSLib.h"

#ifdef SSLIB_USE_YAML

#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

// DEFINES //////////////////////////////////////////////

namespace sstbx {
namespace yaml {

bool insertScalar(YAML::Node & node, const ::std::string & pathValueString);
bool insertScalar(YAML::Node & node, const ::std::string & path, const ::std::string & value);
bool insertScalars(YAML::Node & node, const ::std::vector< ::std::string> & values);

}
}

#endif /* SSLIB_USE_YAML */
#endif /* PROGRAM_OPTIONS_H */

