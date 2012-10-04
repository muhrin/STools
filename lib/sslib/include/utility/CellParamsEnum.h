/*
 * CellParamsEnum.h
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */


#ifndef CELL_PARAMS_ENUM_H
#define CELL_PARAMS_ENUM_H

// INCLUDES /////////////////////////////////////////////

namespace sstbx {
namespace utility {

struct CellParamsEnum { enum { A = 0, B, C, ALPHA, BETA, GAMMA }; };

namespace cell_params_enum {
  enum { A = 0, B, C, ALPHA, BETA, GAMMA };
}
}
}

#endif /* CELL_PARAMS_ENUM_H */
