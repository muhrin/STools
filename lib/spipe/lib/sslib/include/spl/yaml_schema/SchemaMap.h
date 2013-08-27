/*
 * SchemaMap.h
 *
 *
 *  Created on: May 29, 2012
 *      Author: Martin Uhrin
 */

#ifndef SCHEMA_MAP_H
#define SCHEMA_MAP_H

// INCLUDES /////////////////////////////////////////////

#include <map>
#include <string>

#include <boost/ptr_container/ptr_map.hpp>

#include "spl/yaml_schema/detail/SchemaElement.h"
#include "spl/utility/HeterogeneousMap.h"

// DEFINES //////////////////////////////////////////////

namespace spl {
namespace yaml_schema {

// FORWARD DECLARATIONS ////////////////////////////////////
namespace detail {
template <typename T>
class SchemaHeteroMapEntry;
class SchemaHeteroMapEntryBase;
SchemaHeteroMapEntryBase * new_clone(const SchemaHeteroMapEntryBase & entry);
}

template <typename T>
class SchemaHomoMap : public detail::SchemaElementBase< ::std::map< ::std::string, typename T::BindingType> >
{
  // TODO: Test this class and make sure it's doing the right thing
  typedef typename T::BindingType MapSecondType;
public:
  typedef ::std::map< ::std::string, MapSecondType> BindingType;

  SchemaHomoMap();
  virtual ~SchemaHomoMap() {}

  virtual bool valueToNode(YAML::Node & node, const BindingType & value, const bool useDefaultOnFail) const;
  virtual bool nodeToValue(SchemaParse & parse, BindingType & value, const YAML::Node & node, const bool useDefaultOnFail) const;

  void addEntry(const ::std::string & name, const T & element);

  virtual SchemaHomoMap * clone() const;

  bool areUnknownEntriesAllowed() const;
  void setAllowUnknownEntries(const bool allowUnknownEntries);

private:
  typedef ::boost::ptr_map< ::std::string, T> EntriesMap;

  T myDefaultEntry;
  EntriesMap myEntries;
  bool myAllowUnknownEntries;
};

class SchemaHeteroMap : public detail::SchemaElementBase<utility::HeterogeneousMap>
{
public:
  typedef utility::HeterogeneousMap BindingType;

  virtual bool valueToNode(YAML::Node & node, const BindingType & map, const bool useDefaultOnFail) const;
  virtual bool nodeToValue(SchemaParse & parse, BindingType & map, const YAML::Node & node, const bool useDefaultOnFail) const;

  template <typename T>
  detail::SchemaHeteroMapEntry<T> * addEntry(
    const ::std::string & name,
    const utility::Key<T> & key,
    detail::SchemaElementBase<T> * const element
  );
  template <typename T>
  detail::SchemaHeteroMapEntry<T> * addScalarEntry(
    const ::std::string & name,
    const utility::Key<T> & key
  );

  virtual SchemaHeteroMap * clone() const;

private:
  typedef ::boost::ptr_map<const utility::KeyId *, detail::SchemaHeteroMapEntryBase> EntriesMap;

  EntriesMap myEntries;
};


}
}

#include "spl/yaml_schema/detail/SchemaMap.h"

#endif /* SCHEMA_MAP_H */
