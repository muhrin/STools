/*
 * DataTableTest.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "spipetest.h"

#include <limits>
#include <string>

#include <boost/lexical_cast.hpp>

#include <spl/utility/UtilFunctions.h>

// From SPipe
#include <utility/DataTable.h>
#include <utility/DataTableWriter.h>

BOOST_AUTO_TEST_CASE(DataTableTest)
{
  static const size_t NUM_COLUMNS = 10;
  static const size_t NUM_ROWS = 1000;
  static const size_t NUM_RAND_CHARS = 5;
  static const char START_COL = 'a';

  spipe::utility::DataTable table;
  spipe::utility::DataTableWriter writer(table, ::std::string("table.dat"));

  ::std::string key, colName;
  char colChar;
  for(size_t row = 0; row < NUM_ROWS; ++row)
  {
    key = ::boost::lexical_cast< ::std::string>(row);
    for(size_t col = 0; col < NUM_COLUMNS; ++col)
    {
      colChar = START_COL + col;
      table.insert(key, ::std::string(&colChar, &colChar + 1),
          ::spl::utility::generateUniqueName(NUM_RAND_CHARS));
    }
  }

  // NUM_COLUMNS + 1 because of the key which is always column 0
  BOOST_REQUIRE(table.numColumns() == NUM_COLUMNS + 1);
  BOOST_REQUIRE(table.numRows() == NUM_ROWS);

  size_t col = 0;
  spipe::utility::DataTable::ColumnInfoIterator it = table.columnInfoBegin();
  ++it; // Skip key column
  for(spipe::utility::DataTable::ColumnInfoIterator end = table.columnInfoEnd();
      it != end; ++it)
  {
    colChar = START_COL + col;
    BOOST_REQUIRE(::std::string(&colChar, &colChar + 1) == it->getName());
    ++col;
  }
}

