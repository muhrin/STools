/*
 * InfoToken.h
 *
 *
 *  Created on: Nov 19, 2012
 *      Author: Martin Uhrin
 */

#ifndef INFO_TOKEN_H
#define INFO_TOKEN_H

// INCLUDES /////////////////////////////////////////////
#include <string>

#include <utility/TypedDataTable.h>


// FORWARD DECLARES ////////////////////////////////
namespace sstbx {
namespace common {
class Structure;
}
}

namespace stools {
namespace utility {


class InfoToken
{
public:
  typedef const ::sstbx::common::Structure * TableKey;
  typedef ::sstbx::utility::TypedDataTable<TableKey> StructureInfoTable;
  typedef ::sstbx::utility::Column<TableKey> Column;
  typedef StructureInfoTable::SortedKeys SortedKeys;

  InfoToken(const ::std::string & symbol, const ::std::string & defaultFormatString = "");
  virtual ~InfoToken() {}

  virtual ::std::string getName() const = 0;
  virtual bool insert(
    StructureInfoTable & table,
    const ::sstbx::common::Structure & structure) = 0;
  virtual bool remove(StructureInfoTable & table) = 0;
  virtual void sort(SortedKeys & keys, const StructureInfoTable & table) const = 0;
  virtual const Column & getColumn() const = 0;

  const ::std::string & getSymbol() const;
  const ::std::string & getDefaultFormatString() const;

private:
  const ::std::string mySymbol;
  const ::std::string myDefaultFormatString;
};

template <typename T>
class TypedToken : public InfoToken
{
public:
  typedef InfoToken::StructureInfoTable StructureInfoTable;
  typedef InfoToken::SortedKeys SortedKeys;
  typedef InfoToken::TableKey TableKey;
  typedef ::sstbx::utility::Column<TableKey> Column;

  TypedToken(
    const ::std::string & name,
    const ::std::string & symbol,
    const ::std::string & defaultFormatString = "");

  virtual ::std::string getName() const;
  virtual bool insert(
    StructureInfoTable & table,
    const ::sstbx::common::Structure & structure);
  virtual bool remove(StructureInfoTable & table);
  virtual void sort(SortedKeys & keys, const StructureInfoTable & table) const;
  virtual const Column & getColumn() const;

protected:
  typedef ::boost::optional<T> StructureValue;

  virtual StructureValue doGetValue(const ::sstbx::common::Structure & structure) const = 0;

private:
  typedef ::sstbx::utility::TypedColumn<T, TableKey> TypedColumn;

  TypedColumn myColumn;
};

template <typename T>
class StructurePropertyToken : public TypedToken<T>
{
public:
  typedef ::sstbx::utility::Key<T> PropertyKey;

  StructurePropertyToken(
    const ::std::string & name,
    const ::std::string & symbol,
    PropertyKey & propertyKey,
    const ::std::string & defaultFormatString);

protected:
  typedef typename TypedToken<T>::StructureValue StructureValue;

  virtual StructureValue doGetValue(const ::sstbx::common::Structure & structure) const;

private:
  PropertyKey myKey;
};

template <typename T>
class RelativeValueToken : public StructurePropertyToken<T>
{
public:
  typedef ::sstbx::utility::Key<T> PropertyKey;

  RelativeValueToken(
    const ::std::string & name,
    const ::std::string & symbol,
    PropertyKey & propertyKey,
    const ::std::string & defaultFormatString = "",
    const bool usePerAtom = false
  );
  RelativeValueToken(
    const ::std::string & name,
    const ::std::string & symbol,
    PropertyKey & propertyKey,
    const T relativeValue,
    const ::std::string & defaultFormatString = "",
    const bool usePerAtom = false
  );
  void setRelativeTo(const T relativeValue);

protected:
  typedef typename TypedToken<T>::StructureValue StructureValue;

  virtual StructureValue doGetValue(const ::sstbx::common::Structure & structure) const;

private:
  T myRelativeTo; // The value that all values will be relative to
  const bool myUsePerAtom;  // Divide the quantity by the number of atoms
};

template <typename T, typename Getter>
class FunctionToken : public TypedToken<T>
{
public:
  FunctionToken(
    const ::std::string & name,
    const ::std::string & symbol,
    Getter getter,
    const ::std::string & formatString = "");

protected:
  typedef typename TypedToken<T>::StructureValue StructureValue;

  virtual StructureValue doGetValue(const ::sstbx::common::Structure & structure) const;

private:

  Getter myGetter;
};

template <typename T>
::std::auto_ptr<InfoToken>
makeStructurePropertyToken(
  const ::std::string & name,
  const ::std::string & symbol,
  ::sstbx::utility::Key<T> & propertyKey,
  const ::std::string & defaultFormatString = ""
);

template <typename T, typename Getter>
::std::auto_ptr<InfoToken>
makeFunctionToken(
  const ::std::string & name,
  const ::std::string & symbol,
  Getter getter,
  const ::std::string & defaultFormatString = ""
);

}
}

#include "utility/detail/InfoToken.h"

#endif /* INFO_TOKEN_H */
