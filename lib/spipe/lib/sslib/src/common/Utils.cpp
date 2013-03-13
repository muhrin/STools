/*
 * Utils.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: muhrin
 */

// INCLUDES //////////////////////////////////
#include "common/Utils.h"

// NAMESPACES ////////////////////////////////

namespace sstbx {
namespace common {

double randDouble()
{
	return static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
}

double randDouble(const double to)
{
	return randDouble() * to;
}

double randDouble(const double from, const double to)
{
	return randDouble() * (to - from) + from;
}


ProcessId getProcessId()
{
	return NS_BOOST_IPC_DETAIL::get_current_process_id();
}

}
}
