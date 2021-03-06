/*
 * PipeRunnerListener.h
 *
 *
 *  Created on: Feb 17, 2012
 *      Author: Martin Uhrin
 */

#ifndef PIPE_RUNNER_LISTENER_H
#define PIPE_RUNNER_LISTENER_H

// INCLUDES /////////////////////////////////////////////


namespace pipelib {
namespace event {

// FORWARD DECLARATIONS ////////////////////////////////////

template <class Runner>
class PipeRunnerDestroyed;
template <class Runner>
class PipeRunnerStateChanged;

template <class Runner>
class PipeRunnerListener
{
public:
  virtual void notify(const PipeRunnerStateChanged<Runner> & evt) {}
  virtual void notify(const PipeRunnerDestroyed<Runner> & evt) {}

};

}
}

#endif /* PIPE_RUNNER_LISTENER_H */
