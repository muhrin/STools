/*
 * BlockLoaderType.cpp
 *
 *  Created on: Jan 13, 2014
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "io/BlockLoaderType.h"

#include <boost/tokenizer.hpp>

#include "build/BlockLoader.h"
// NAMESPACES ////////////////////////////////

namespace spipe {
namespace io {

typedef boost::tokenizer< boost::char_separator< char> > Tok;
static const boost::char_separator< char> SEP("| ");

bool
BlockLoaderType::nodeToValue(const YAML::Node & node, BlockHandle * const block,
    schemer::ParseLog * const log) const
{
  if(!node.IsMap() || !node["run"].IsScalar())
  {
    return false;
  }

  build::BlockLoader loader;
  loader.load(node);

  BlockHandle startBlock, last;
  Tok tok(node["run"].Scalar(), SEP);

  for(Tok::const_iterator it = tok.begin(), end = tok.end(); it != end; ++it)
  {
    BlockHandle temp = loader.get(*it);
    if(!temp.get())
    {
      log->logError(schemer::ParseLogErrorCode::REQUIRED_VALUE_MISSING,
          "Failed to find block " + *it);
      return false;
    }

    if(it == tok.begin())
      startBlock = last = temp;
    else
    {
      last->connect(temp);
      last = temp;
    }
  }
  *block = startBlock;

  return true;
}

} // namespace io
} // namespace spipe
