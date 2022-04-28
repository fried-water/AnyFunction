#pragma once

#include "traits.h"

#include <vector>

namespace anyf {

using TypeID = intptr_t;

template <typename T>
constexpr TypeID type_id() {
  return reinterpret_cast<TypeID>(&type_id<T>);
}

class Type {
  constexpr static uint8_t CONST_FLAG = 1 << 0;
  constexpr static uint8_t REF_FLAG = 1 << 1;
  constexpr static uint8_t COPY_FLAG = 1 << 2;
  constexpr static uint8_t MOVE_FLAG = 1 << 3;

public:
  Type() = default;

  template <typename T>
  constexpr Type(Ty<T>) : _type(anyf::type_id<std::decay_t<T>>()) {
    if constexpr(std::is_const_v<std::remove_reference_t<T>>)
      _properties |= Type::CONST_FLAG;
    if constexpr(std::is_reference_v<T>)
      _properties |= Type::REF_FLAG;
    if constexpr(std::is_copy_constructible_v<T>)
      _properties |= Type::COPY_FLAG;
    if constexpr(std::is_move_constructible_v<T>)
      _properties |= Type::MOVE_FLAG;
  }

  constexpr Type decayed() const {
    constexpr uint8_t decayed_mask = Type::COPY_FLAG | Type::MOVE_FLAG;
    Type d;
    d._type = _type;
    d._properties = _properties & decayed_mask;
    return d;
  }

  constexpr TypeID type_id() const { return _type; }

  constexpr bool is_const() const { return (_properties & CONST_FLAG) > 0; }
  constexpr bool is_ref() const { return (_properties & REF_FLAG) > 0; }
  constexpr bool is_copy_constructible() const { return (_properties & COPY_FLAG) > 0; }
  constexpr bool is_move_constructible() const { return (_properties & MOVE_FLAG) > 0; }

  constexpr friend bool operator==(const Type& lhs, const Type& rhs) {
    return lhs.type_id() == rhs.type_id() && lhs._properties == rhs._properties;
  }

  constexpr friend bool operator!=(const Type& lhs, const Type& rhs) { return !(lhs == rhs); }

private:
  TypeID _type = 0;
  uint8_t _properties = 0;
};

template <typename... Ts>
std::vector<Type> make_types(TL<Ts...>) {
  return {Type(Ty<Ts>{})...};
}

} // namespace anyf
