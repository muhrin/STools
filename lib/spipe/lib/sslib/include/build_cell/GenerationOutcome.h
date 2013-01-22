/*
 * GenerationOutcome.h
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef GENERATION_OUTCOME_H
#define GENERATION_OUTCOME_H

// INCLUDES /////////////////////////////////


// FORWARD DECLARES //////////////////////////

namespace sstbx {
namespace build_cell {

class GenerationOutcome
{
public:
  GenerationOutcome(): mySucceeded(false) {}

  bool success() const;
  const ::std::string & getErrorMessage() const;

  void setSuccess()
  {
    mySucceeded = true;
  }

  void setFailure(const ::std::string & msg = "")
  {
    mySucceeded = false;
    myErrorMessage = msg;
  }

private:
  bool mySucceeded;
  ::std::string myErrorMessage;
};

inline bool GenerationOutcome::success() const
{
  return mySucceeded;
}

}
}

#endif /* GENERATION_OUTCOME_H */
