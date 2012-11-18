/*
 * UtilityFunctions.h
 *
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef UTILITY_FUNCTIONS_H
#define UTILITY_FUNCTIONS_H

// INCLUDES /////////////////////////////////////////////

#include <string>

// Tell boost not to try to link to the date_time library as interprocess
// only uses a portion that doesn't need compilation.
// See: http://www.boost.org/doc/libs/1_49_0/doc/html/interprocess.html#interprocess.intro.introduction_building_interprocess
#define BOOST_DATE_TIME_NO_LIB

#include <boost/interprocess/detail/os_thread_functions.hpp>
#include <boost/version.hpp>
#include <boost/any.hpp>

// FORWARD DECLARES ////////////////////////////////
namespace spipe {
namespace common {
class StructureData;
}
}

namespace sstbx {
namespace io {
struct AdditionalData;
}
}

// DEFINES ////////////////////////////////////////
#define EPSILON_REL 1e-5;

#if (BOOST_VERSION / 100000) <= 1 && ((BOOST_VERSION / 100) % 1000) <= 47
	#define	NS_BOOST_IPC_DETAIL boost::interprocess::detail
#else
	#define NS_BOOST_IPC_DETAIL boost::interprocess::ipcdetail
#endif

#define ProcessId NS_BOOST_IPC_DETAIL::OS_process_id_t

// FUNCTIONS ////////////////////////////////////////

namespace spipe {
namespace common {

ProcessId getProcessId();

std::string generateUniqueName();

void parseParamString(
  const std::string & str,
  double &            from,
  double &            step,
  unsigned int &      nSteps
);

::std::string getString(const double in, unsigned int digitsAfterDecimal = 5);

}
}

#endif /* UTILITY_FUNCTIONS_H */
