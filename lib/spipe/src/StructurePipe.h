/*
 * StructurePipe.h
 *
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef STRUCTURE_PIPE_H
#define STRUCTURE_PIPE_H

#ifndef NULL
#define NULL 0
#endif

#define DEBUG 1


// Include the configuration header file
#include "StructurePipeConfig.h"

#if !DEBUG
	#define SP_ASSERT(x)
#else
  #include <iostream>

	#define SP_ASSERT(x) \
	if (! (x)) \
	{ \
		std::cout << "ERROR!! Assert " << #x << " failed\n"; \
		std::cout << " on line " << __LINE__  << "\n"; \
		std::cout << " in file " << __FILE__ << "\n";  \
	}
#endif
		// Create a breakpoint
		//__asm { \
		//	int 3 \
		//} \


// INCLUDES /////////////


// FORWARD DECLARES //////////////
namespace pipelib
{
template <
  typename PipelineData,
  typename SharedData,
  typename GlobalData
>
class Block;

template <
  typename PipelineData,
  typename SharedData,
  typename GlobalData
>
class IPipeline;

template <
  typename PipelineData,
  typename SharedData,
  typename GlobalData
>
class PipeBlock;

template <
  typename PipelineData,
  typename SharedData,
  typename GlobalData
>
class StartBlock;

}

namespace spipe {
namespace common {
class SharedData;
class StructureData;
}
}


namespace spipe {

// TYPEDEFS ///////////////////////

typedef spipe::common::StructureData			                                    StructureDataTyp;
typedef spipe::common::SharedData					                                    SharedDataTyp;
typedef pipelib::IPipeline<StructureDataTyp, SharedDataTyp, SharedDataTyp>  	SpPipelineTyp;
typedef pipelib::Block<StructureDataTyp, SharedDataTyp, SharedDataTyp>        SpBlockTyp;
typedef pipelib::PipeBlock<StructureDataTyp, SharedDataTyp, SharedDataTyp>    SpPipeBlockTyp;
typedef pipelib::StartBlock<StructureDataTyp, SharedDataTyp, SharedDataTyp>   SpStartBlockTyp;

}

#endif /* STRUCTURE_PIPE_H */
