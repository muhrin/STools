/*
 * YamlOptionsParser.h
 *
 *
 *  Created on: Nov 19, 2012
 *      Author: Martin Uhrin
 */

#ifndef YAML_OPTIONS_PARSER_H
#define YAML_OPTIONS_PARSER_H

// INCLUDES /////////////////////////////////////////////

#include <boost/program_options/parsers.hpp>

#include <yaml-cpp/yaml.h>

// FORWARD DECLARES ////////////////////////////////
namespace stools {
namespace utility {

template <class charT>
class BasicYamlOptionsParser
{

  BasicYamlOptionsParser(const YAML::Node & root);

  /** Sets options descriptions to use. */
  BasicYamlOptionsParser & options(const ::boost::program_options::options_description & desc);

  /** Parses the options and returns the result of parsing.
      Throws on error.
  */
  ::boost::program_options::basic_parsed_options<charT> run();

private:

  const ::boost::program_options::options_description & myDescription;
  const YAML::Node & myRoot;

};

// IMPLEMENTATION ////////////////
template <class charT>
::boost::program_options::basic_parsed_options<charT>
parseYamlOptions(
  const YAML::Node & root,
  const ::boost::program_options::options_description & options);

template <class charT>
BasicYamlOptionsParser<charT>::BasicYamlOptionsParser(const YAML::Node &root):
myRoot(root)
{}


}
}

#endif /* YAML_OPTIONS_PARSER_H */
