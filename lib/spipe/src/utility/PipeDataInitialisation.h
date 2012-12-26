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
#include <SSLibTypes.h>

#include <SpTypes.h>

// FORWARD DECLARES ////////////////////////////////
namespace spipe {
namespace common {
class GlobalData;
class SharedData;
}
}

// DEFINES ////////////////////////////////////////


// FUNCTIONS ////////////////////////////////////////

namespace spipe {
namespace common {

::spipe::SpEngine::RunnerPtr generateRunnerInitDefault(::spipe::SpEngine & engine);

void initGlobalDataDefault(GlobalData & global);


}
}

#endif /* PIPE_DATA_INITIALISATION_H */
