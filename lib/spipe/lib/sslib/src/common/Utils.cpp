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
	return ((double)rand() / RAND_MAX);
}

double randDouble(const double to)
{
	return ((double)rand() / RAND_MAX) * to;
}

double randDouble(const double from, const double to)
{
	return ((double)rand() / RAND_MAX) * (to - from) + from;
}


ProcessId getProcessId()
{
	return NS_BOOST_IPC_DETAIL::get_current_process_id();
}

}
}
