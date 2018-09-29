#ifndef TYPE_H
#define TYPE_H

#include "util.h"

#include <iostream>
#include <tuple>
#include <type_traits>
#include <typeindex>
#include <typeinfo>

namespace anyf {

class Type {
  bool _is_const;
  bool _is_trivially_copyable;
  std::type_index _index;

public:
  Type(bool is_const, bool is_trivially_copyable, std::type_index index)
      : _is_const(is_const), _is_trivially_copyable(is_trivially_copyable),
        _index(index) {}

  bool is_const() const { return _is_const; }
  bool is_trivially_copyable() const { return _is_trivially_copyable; }
  std::type_index type_index() const { return _index; }

  const char* name() const { return _index.name(); }

  friend bool operator==(Type x, Type y) { return x._index == y._index; }

  friend bool operator!=(Type x, Type y) { return !(x == y); }

  friend std::ostream& operator<<(std::ostream& out, const Type& x) {
    return out << x._index.name();
  }
};

template <typename T>
inline Type make_type() {
  return Type(std::is_const_v<T>, std::is_trivially_copyable_v<T>,
              std::type_index(typeid(T)));
}

template <typename TupleTypes, std::size_t... Is>
inline small_vec<Type, 3> make_types_impl(std::index_sequence<Is...>) {
  return util::make_vector<small_vec<Type, 3>>(
      make_type<std::tuple_element_t<Is, TupleTypes>>()...);
}

template <typename TupleTypes>
inline small_vec<Type, 3> make_types() {
  return make_types_impl<TupleTypes>(
      std::make_index_sequence<std::tuple_size_v<TupleTypes>>());
}

} // namespace anyf

#endif