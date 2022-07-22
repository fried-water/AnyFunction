#pragma once

#include "anyf/traits.h"

#include <vector>

namespace anyf {

using TypeID = uintptr_t;

template <typename T>
constexpr TypeID type_id(Type<T>) {
  static_assert(std::is_same_v<T, std::decay_t<T>>);
  static_assert(std::is_move_constructible_v<T>);

  return reinterpret_cast<TypeID>(&type_id<T>) | uintptr_t(std::is_copy_constructible_v<T>);
}

template <typename T>
constexpr TypeID type_id() { return type_id(Type<T>{}); }

constexpr bool is_copyable(TypeID t) { return t & 1; }

struct TypeProperties {
  TypeID id = {};
  bool value = {};

  KNOT_ORDERED(TypeProperties);
};

template <typename T>
constexpr TypeProperties make_type_properties(Type<T> t) {
  constexpr bool is_value = std::is_same_v<T, std::decay_t<T>>;
  constexpr bool is_rref = std::is_same_v<T, std::decay_t<T>&&>;
  constexpr bool is_cref = std::is_same_v<T, const std::decay_t<T>&>;

  static_assert(is_value || is_rref || is_cref);

  return TypeProperties{type_id(decay(t)), is_value | is_rref};
}

template <typename... Ts>
std::vector<TypeProperties> make_types(TypeList<Ts...>) {
  return {make_type_properties(Type<Ts>{})...};
}

} // namespace anyf
