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

#include <map>
#include <string>
#include <vector>

#ifdef SP_ENABLE_THREAD_AWARE
#  include <boost/thread/mutex.hpp>
#endif

#include <pipelib/event/EventSupport.h>

#include "StructurePipe.h"

// FORWARD DECLARATIONS ////////////////////////////////////

namespace spipe {
namespace utility {

class IDataTableChangeListener;

class DataTable
{
public:
  typedef ::std::string Key;
  typedef ::std::string Value;

  class Column
  {
  public:

    Column();
    Column(const ::std::string & name);
    Column(const char * const name);
    Column(const Column & toCopy);

    bool
    operator ==(const Column & rhs) const;

    const ::std::string &
    getName() const;

  private:
    ::std::string myName;

    friend class DataTable;
  };

private:
  typedef ::std::vector< Column> ColumnInfo;
  typedef ::std::vector< Value> ColumnData;
  // Maps a row key to column data
  typedef ::std::map< Key, size_t> RowMap;
  typedef ::std::vector<ColumnData> Rows;
  typedef ::std::vector< ::std::string> NotesContainer;

public:
  typedef Rows::const_iterator RowIterator;
  typedef ColumnInfo::const_iterator ColumnInfoIterator;
  typedef NotesContainer::const_iterator NotesIterator;

  static const int COL_UNDEFINED = -1;

  DataTable();

  RowIterator rowsBegin() const;
  RowIterator rowsEnd() const;

  ColumnInfoIterator columnInfoBegin() const;
  ColumnInfoIterator columnInfoEnd() const;

  NotesIterator notesBegin() const;
  NotesIterator notesEnd() const;

  //void insert(const Key & key, const size_t col, const Value & value);
  size_t
  insert(const Key & key, const Column & col, const Value & value);

  void
  addTableNote(const ::std::string & note);

  void
  writeToFile(const ::std::string & filename,
      const ::std::string & colDelimiter = "\t") const;

  void
  clear();

  // Event //////////////////////////////////////
  void
  addDataTableChangeListener(IDataTableChangeListener & listener);
  bool
  removeDataTableChangeListener(IDataTableChangeListener & listener);

private:
  typedef ::pipelib::event::EventSupport< IDataTableChangeListener> ChangeListenerSupport;

  void
  insertColumn(const Column & colInfo, const size_t col);
  Value
  insertValue(const Key & key, const size_t col, const Value & value);

  ColumnInfo myColumns;
  Rows myRows;
  RowMap myRowMap;
  NotesContainer myTableNotes;

  ChangeListenerSupport myChangeListenerSupport;

#ifdef SP_ENABLE_THREAD_AWARE
  ::boost::mutex myTableMutex;
  ::boost::mutex myNotesMutex;
#endif

};

}
}

#endif /* DATA_TABLE_H */
