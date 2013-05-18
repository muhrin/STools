/*
 * CastepGeomJob.h
 *
 *
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef CASTEP_GEOM_JOB_H
#define CASTEP_JOB_H

// INCLUDES /////////////////////////////////////////////
#include <string>
#include <vector>

#include <boost/scoped_ptr.hpp>

#include "os/Process.h"

// DEFINES //////////////////////////////////////////////

namespace sstbx {
// FORWARD DECLARATIONS ////////////////////////////////////

namespace potential {
class CastepRun;
class ICastepJobListener;

class CastepJob
{
public:
  virtual os::Process::RunResult::Value run();
private:
  typedef ::std::vector< ::std::string> Args;

  CastepJob(const ::std::string & runCommand, CastepRun & castepRun);

  CastepRun & myCastepRun;
  ::boost::scoped_ptr<os::Process> myProcess;
  Args myArgs;

  friend class CastepRun;
};


}
}

#endif /* CASTEP_JOB_H */
