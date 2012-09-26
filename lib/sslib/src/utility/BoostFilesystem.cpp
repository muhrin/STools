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

#ifndef SSLIB_USE_BOOSTFS_V2
template < >
::boost::filesystem::path &
::boost::filesystem::path::append< ::boost::filesystem::path::iterator >(
  path::iterator begin,
  path::iterator end)
{ 
  for( ; begin != end ; ++begin )
    *this /= *begin;
  return *this;
}
#endif

// Return path when appended to a_From will resolve to same as a_To
boost::filesystem::path make_relative(
  boost::filesystem::path a_From,
  boost::filesystem::path a_To )
{
#ifdef SSLIB_USE_BOOSTFS_V2
  a_From = ::boost::filesystem::system_complete( a_From );
  a_To = ::boost::filesystem::system_complete( a_To );
#else
  a_From = ::boost::filesystem::absolute( a_From );
  a_To = ::boost::filesystem::absolute( a_To );
#endif

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
#ifdef SSLIB_USE_BOOSTFS_V2
  ret = ::sstbx::utility::append(ret, itrTo, a_To.end());
#else
  ret.append( itrTo, a_To.end() );
#endif
  return ret;
}
}
}

namespace sstbx
{
namespace utility
{

namespace fs = ::boost::filesystem;

::std::string stemString(const fs::path & path)
{
#ifdef SSLIB_USE_BOOSTFS_V2
  return path.stem();
#else
  return path.stem().string();
#endif
}


fs::path
append(
  fs::path appendTo,
  fs::path::iterator begin,
  fs::path::iterator end
)
{
  for( ; begin != end ; ++begin )
    appendTo /= *begin;
  return appendTo;
}

::boost::filesystem::path
absolute(const ::boost::filesystem::path & p)
{
#ifdef SSLIB_USE_BOOSTFS_V2
  return fs::system_complete(p);
#else
  return fs::absolute(p);
#endif
}

bool
isAbsolute(const ::boost::filesystem::path & toCheck)
{
#ifdef SSLIB_USE_BOOSTFS_V2
  return toCheck.is_complete();
#else
  return toCheck.is_absolute();
#endif
}

}
}
