/*
 * BoostFilesystem.cpp
 *
 *  Created on: Aug 17, 2011
 *      Author: muhrin
 */

// INCLUDES /////////////////////////////////////
#include "utility/BoostFilesystem.h"

namespace boost
{
namespace filesystem3
{

template <>
path & path::append< typename path::iterator >(
  typename path::iterator begin,
  typename path::iterator end,
  const codecvt_type& cvt)
{ 
  for( ; begin != end ; ++begin )
    *this /= *begin;
  return *this;
}

// Return path when appended to a_From will resolve to same as a_To
boost::filesystem::path make_relative(
  boost::filesystem::path a_From,
  boost::filesystem::path a_To )
{
  a_From = boost::filesystem::absolute( a_From ); a_To = boost::filesystem::absolute( a_To );
  boost::filesystem::path ret;
  boost::filesystem::path::const_iterator itrFrom( a_From.begin() ), itrTo( a_To.begin() );
  
  // Find common base
  for(boost::filesystem::path::const_iterator toEnd( a_To.end() ), fromEnd( a_From.end() ) ;
    itrFrom != fromEnd && itrTo != toEnd && *itrFrom == *itrTo;
    ++itrFrom, ++itrTo );

  // Navigate backwards in directory to reach previously found base
  for( boost::filesystem::path::const_iterator fromEnd( a_From.end() ); itrFrom != fromEnd; ++itrFrom )
  {
    if( (*itrFrom) != "." )
      ret /= "..";
  }
  // Now navigate down the directory branch
  ret.append( itrTo, a_To.end() );
  return ret;
}
}
}

namespace sstbx
{
namespace utility
{

::std::string stemString(const ::boost::filesystem::path & path)
{
#ifdef SSLIB_USE_BOOSTFS_V2
  return path.stem();
#else
  return path.stem().string();
#endif
}


}
}
