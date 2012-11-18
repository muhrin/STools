/*
 * DataLocation.h
 *
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef DATA_LOCATION_H
#define DATA_LOCATION_H

// INCLUDES /////////////////////////////////////////////

// FORWARD DECLARATIONS ////////////////////////////////////
namespace spipe
{
namespace common
{

struct DataLocation
{
  enum Value
  {
    NONE,
    STRUCTURE,
    SHARED,
    GLOBAL
  };
};

}
}

#endif /* DATA_LOCATION_H */

