/*
 * PipeDataInitialisation.h
 *
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef PIPE_DATA_INITIALISATION_H
#define PIPE_DATA_INITIALISATION_H

// INCLUDES /////////////////////////////////////////////
#include <spl/SSLibTypes.h>

#include <SpTypes.h>

// FORWARD DECLARES ////////////////////////////////
namespace spipe {
namespace io{
class StructureReadWriteManager;
}
}

// DEFINES ////////////////////////////////////////


// FUNCTIONS ////////////////////////////////////////

namespace spipe {
namespace utility {

void initStructureRwManDefault(::spl::io::StructureReadWriteManager & rwMan);


}
}

#endif /* PIPE_DATA_INITIALISATION_H */
