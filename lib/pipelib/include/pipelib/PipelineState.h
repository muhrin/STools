/*
 * PipelineState.h
 *
 *
 *  Created on: Feb 17, 2012
 *      Author: Martin Uhrin
 */

#ifndef PIPELINE_STATE_H
#define PIPELINE_STATE_H

// INCLUDES /////////////////////////////////////////////

// FORWARD DECLARATIONS ////////////////////////////////////

namespace pipelib
{

struct PipelineState
{
  enum Value
  {
    // The pipeline has not been initialised yet, changes can be made
    UNINITIALISED,
    // The pipeline is being initialised
    INITIALISING,
    // The pipeline has been initialised but has never been started
    INITIALISED,
    // The pipeline is running
    RUNNING,
    // The pipeline has stopped
    STOPPED,
    // The pipeline has run through once and has been reset back to the beginning
    RESET
  };
};



}

#endif /* PIPELINE_STATE_H */
