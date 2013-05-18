/*
 * CastepJob.h
 *
 *
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef CASTEP_JOB_H
#define CASTEP_JOB_H

// INCLUDES /////////////////////////////////////////////
#include <string>
#include <vector>

#include <boost/filesystem/fstream.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/optional.hpp>
#include <boost/scoped_ptr.hpp>

#include "os/Process.h"

// DEFINES //////////////////////////////////////////////

namespace sstbx {
// FORWARD DECLARATIONS ////////////////////////////////////

namespace potential {
class CastepRun;
class ICastepGeomOptimisationListener;


class CastepJob
{
public:
  virtual os::Process::RunResult::Value runBlocking();
  virtual os::Process::RunResult::Value run();
  bool stop();

protected:
  CastepJob(const ::std::string & runCommand, const CastepRun & castepRun);

  os::Process::RunResult::Value doRunBlocking();
  os::Process::RunResult::Value doRun();

  const CastepRun & getCastepRun() const;
  os::Process & getProcess();
  const os::Process & getProcess() const;
  
private:
  typedef ::std::vector< ::std::string> Args;
  
  const CastepRun & myCastepRun;
  ::boost::scoped_ptr<os::Process> myProcess;  
  Args myArgs;

  friend class CastepRun;
};


class CastepGeomJob : public CastepJob
{
public:
  virtual os::Process::RunResult::Value run();

  void setOptimisationListener(ICastepGeomOptimisationListener * listener);
  const ICastepGeomOptimisationListener * getListener() const;

private:
  CastepGeomJob(const ::std::string & runCommand, const CastepRun & castepRun);

  bool waitForCastepFile(const ::boost::filesystem::path & castepFile) const;
  ::boost::optional<int> seekNextStep(::boost::filesystem::ifstream & castepStream) const;

  ICastepGeomOptimisationListener * myOptimisationListener;

  friend class CastepRun;
};

class ICastepGeomOptimisationListener
{
public:
  virtual void finishedStep(const int step) = 0;
};

}
}

#endif /* CASTEP_JOB_H */
