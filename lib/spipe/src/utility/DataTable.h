/*
 * DataTable.h
 *
 *
 *  Created on: Aug 17, 2011
 *      Author: Martin Uhrin
 */

#ifndef DATA_TABLE_H
#define DATA_TABLE_H

// INCLUDES /////////////////////////////////////////////
#include "StructurePipe.h"

#include <map>
#include <string>
#include <utility>
#include <vector>

#ifdef SP_ENABLE_THREAD_AWARE
#  include <boost/thread/mutex.hpp>
#endif

// FORWARD DECLARATIONS ////////////////////////////////////

namespace spipe {
namespace utility {

class IDataTableChangeListener;

class DataTable
{
  static const ::std::string KEY_COLUMN_NAME;
  static const ::std::vector< ::std::string> DEFAULT_COLUMN_NAMES;
public:
  typedef ::std::string Key;
  typedef ::std::string Value;

private:
  typedef ::std::vector< Value> Row;
  // Maps a row key to column data
  typedef ::std::map< Key, size_t> RowMap;
  typedef ::std::vector< ::std::string> ColumnNames;
  typedef ::std::map< ::std::string, size_t> ColumnMap;
  typedef ::std::vector<Row> Rows;
  typedef ::std::vector< ::std::string> NotesContainer;

public:
  typedef ::std::pair<size_t, size_t> Coords;
  typedef Rows::const_iterator RowIterator;
  typedef ColumnNames::const_iterator ColumnNameIterator;
  typedef NotesContainer::const_iterator NotesIterator;

  static const int COL_UNDEFINED = -1;

  DataTable();

  size_t numRows() const;
  RowIterator rowsBegin() const;
  RowIterator rowsEnd() const;

  size_t numColumns() const;
  ColumnNameIterator columnNamesBegin() const;
  ColumnNameIterator columnNamesEnd() const;

  bool hasNotes() const;
  NotesIterator notesBegin() const;
  NotesIterator notesEnd() const;

  Coords
  insert(const Key & key, const ::std::string & colName, const Value & value);

  void
  addTableNote(const ::std::string & note);

  void
  writeToFile(const ::std::string & filename,
      const ::std::string & colDelimiter = " ") const;

  bool
  empty() const;
  void
  clear();

  // Event //////////////////////////////////////
  void
  addDataTableChangeListener(IDataTableChangeListener * const listener);
  bool
  removeDataTableChangeListener(IDataTableChangeListener * const listener);

private:
  Value
  insertValue(const Coords & coords, const Value & value);
  size_t
  row(const Key & key);
  size_t
  col(const ::std::string & colName);
  Coords
  coords(const Key & key, const ::std::string & colName);

  Rows myRows;
  RowMap myRowMap;
  ColumnNames myColumnNames;
  ColumnMap myColumnMap;
  NotesContainer myTableNotes;

  ::std::set<IDataTableChangeListener *> myListeners;

#ifdef SP_ENABLE_THREAD_AWARE
  ::boost::mutex myTableMutex;
  ::boost::mutex myNotesMutex;
#endif
};

}
}

#endif /* DATA_TABLE_H */
