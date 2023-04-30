#pragma once

#include "anyf/traits.h"

#include "knot/core.h"

#include <vector>

namespace anyf {

struct TypeID {
  uintptr_t id = {};
  KNOT_ORDERED(TypeID);
};

template <typename T>
constexpr TypeID type_id(Type<T>) {
  static_assert(std::is_same_v<T, std::decay_t<T>>);
  static_assert(std::is_move_constructible_v<T>);

  return { reinterpret_cast<uintptr_t>(&type_id<T>) | uintptr_t(std::is_copy_constructible_v<T>) };
}

template <typename T>
constexpr TypeID type_id() {
  return type_id(Type<T>{});
}

constexpr bool is_copyable(TypeID t) { return t.id & 1; }

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
std::vector<TypeProperties> make_type_properties(TypeList<Ts...>) {
  return {make_type_properties(Type<Ts>{})...};
}

template <typename... Ts>
std::vector<TypeID> make_type_ids(TypeList<Ts...>) {
  return {type_id(decay(Type<Ts>{}))...};
}

} // namespace anyf

template<>
struct std::hash<anyf::TypeID> {
  size_t operator()(anyf::TypeID t) const noexcept {
    return std::hash<uintptr_t>{}(t.id);
  }
};
