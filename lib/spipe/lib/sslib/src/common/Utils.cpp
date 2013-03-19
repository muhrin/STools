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

ProcessId getProcessId()
{
	return NS_BOOST_IPC_DETAIL::get_current_process_id();
}

}
}
