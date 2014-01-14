/*
 * BlockLoader.cpp
 *
 *  Created on: Dec 13, 2013
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "build/PipeBuilder.h"

#include <boost/algorithm/string/trim.hpp>
#include <boost/tokenizer.hpp>

#include "build/BlockLoader.h"

// NAMESPACES ////////////////////////////////

namespace spipe {
namespace build {

typedef boost::tokenizer< boost::char_separator< char> > Tok;
static const boost::char_separator< char> SEP("|");

BlockHandle
buildPipe(const YAML::Node & node)
{
  if(!node.IsMap() || !node["run"].IsScalar())
  {
    std::cerr << "Error: Must specify run entry in node\n";
    return BlockHandle();
  }

  BlockLoader blockLoader;
  blockLoader.load(node);

  BlockHandle startBlock, last;
  const Tok tok(node["run"].Scalar(), SEP);
  for(Tok::const_iterator it = tok.begin(), end = tok.end(); it != end; ++it)
  {
    BlockHandle temp = blockLoader.get(boost::algorithm::trim_copy(*it));
    if(!temp)
    {
      std::cerr << "Error: Failed to find block " << *it << "\n";
      return BlockHandle();
    }

    if(it == tok.begin())
      startBlock = last = temp;
    else
    {
      last->connect(temp);
      last = temp;
    }
  }
  return startBlock;
}

}
}
