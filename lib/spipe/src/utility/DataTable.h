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

#include <pipelib/event/EventSupport.h>

// FORWARD DECLARATIONS ////////////////////////////////////

namespace spipe
{
namespace utility
{

class IDataTableChangeListener;

class DataTable
{
public:

  typedef ::std::string   Key;
  typedef ::std::string   Value;

  static const int COL_UNDEFINED = -1;

  class Column
  {
  public:

    Column();
    Column(const ::std::string & name);
    Column(const char * const name);
    Column(const Column & toCopy);

    bool operator ==(const Column & rhs) const;


    const ::std::string & getName() const;

  private:

    ::std::string myName;

    friend class DataTable;
  };

  //void insert(const Key & key, const size_t col, const Value & value);
  size_t insert(const Key & key, const Column & col, const Value & value);

  void addTableNote(const ::std::string & note);

  void writeToFile(
    const ::std::string & filename,
    const ::std::string & colDelimiter = " ") const;

  void clear();

  // Event //////////////////////////////////////
  void addDataTableChangeListener(IDataTableChangeListener & listener);
  bool removeDataTableChangeListener(IDataTableChangeListener & listener);

private:

  /* Maps each column to a value*/
  typedef ::std::vector<Value>             ColumnData;
  /* Maps a row key to column data */
  typedef ::std::map<Key, ColumnData>      RowMap;

  typedef ::std::vector<Column>            ColumnInfo;

  typedef ::pipelib::event::EventSupport<IDataTableChangeListener> ChangeListenerSupport;
  typedef ::std::vector< ::std::string>    NotesContainer;

  void insertColumn(const Column & colInfo, const size_t col);
  Value insertValue(const Key & key, const size_t col, const Value & value);

  ::std::string         myColDelimiter;
  ColumnInfo            myColumns;
  RowMap                myRows;
  NotesContainer        myTableNotes;


  ChangeListenerSupport myChangeListenerSupport;

  friend class DataTableWriter;
};


}}

#endif /* DATA_TABLE_H */
