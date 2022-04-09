#pragma once

#include "traits.h"

#include <vector>

namespace anyf {

class TypeId {
  intptr_t _id = {};
  constexpr TypeId(intptr_t id) : _id{id} {}

public:
  constexpr TypeId() = default;

  template <typename T>
  friend constexpr TypeId type_id();

  friend constexpr bool operator==(TypeId lhs, TypeId rhs) { return lhs._id == rhs._id; }
  friend constexpr bool operator!=(TypeId lhs, TypeId rhs) { return !(lhs == rhs); }
};

template <typename T>
constexpr TypeId type_id() {
  return {reinterpret_cast<intptr_t>(&type_id<T>)};
}

class Type {
  constexpr static uint8_t CONST_FLAG = 1 << 0;
  constexpr static uint8_t REF_FLAG = 1 << 1;
  constexpr static uint8_t COPY_FLAG = 1 << 2;
  constexpr static uint8_t MOVE_FLAG = 1 << 3;

public:
  Type() = default;

  constexpr TypeId type_id() const { return _type; }

  constexpr bool is_const() const { return (_properties & CONST_FLAG) > 0; }
  constexpr bool is_ref() const { return (_properties & REF_FLAG) > 0; }
  constexpr bool is_copy_constructible() const { return (_properties & COPY_FLAG) > 0; }
  constexpr bool is_move_constructible() const { return (_properties & MOVE_FLAG) > 0; }

  constexpr friend bool operator==(const Type& lhs, const Type& rhs) {
    return lhs.type_id() == rhs.type_id() && lhs._properties == rhs._properties;
  }

  constexpr friend bool operator!=(const Type& lhs, const Type& rhs) { return !(lhs == rhs); }

private:
  TypeId _type;
  uint8_t _properties = 0;

  template <typename T>
  friend constexpr Type make_type();

  constexpr Type(TypeId type, uint8_t properties) : _type(type), _properties(properties) {}
};

template <typename T>
constexpr Type make_type() {
  uint8_t properties = 0;

  if constexpr(std::is_const_v<std::remove_reference_t<T>>)
    properties |= Type::CONST_FLAG;
  if constexpr(std::is_reference_v<T>)
    properties |= Type::REF_FLAG;
  if constexpr(std::is_copy_constructible_v<T>)
    properties |= Type::COPY_FLAG;
  if constexpr(std::is_move_constructible_v<T>)
    properties |= Type::MOVE_FLAG;

  return Type(type_id<std::decay_t<T>>(), properties);
}

template <typename... Ts>
std::vector<Type> make_types(TL<Ts...>) {
  return {make_type<Ts>()...};
}

} // namespace anyf
