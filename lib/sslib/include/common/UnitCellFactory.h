/*
 * UnitCellFactory.h
 *
 * 
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef UNIT_CELL_FACTORY_H
#define UNIT_CELL_FACTORY_H

// INCLUDES /////////////////////////////////////////////

// FORWARD DECLARATIONS ////////////////////////////////////
namespace sstbx
{
namespace common
{
class AbstractFmidCell;
}
}

namespace sstbx { namespace common {

class UnitCellFactory
{
public:
	
	AbstractFmidCell * createCell(
		const double a, const double b, const double c,
		const double alpha, const double beta, const double gamma) const;

	static const UnitCellFactory * inst();

protected:
	UnitCellFactory() {};

private:

	static UnitCellFactory * instance;
};

}}

#endif /* UNIT_CELL_FACTORY_H */
