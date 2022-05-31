#pragma once

#include "anyf/traits.h"

#include <vector>

namespace anyf {

using TypeID = intptr_t;

template <typename T>
constexpr TypeID type_id(Type<T>) {
  return reinterpret_cast<TypeID>(&type_id<T>);
}

class TypeProperties {
  constexpr static uint8_t CONST_FLAG = 1 << 0;
  constexpr static uint8_t LREF_FLAG = 1 << 1;
  constexpr static uint8_t RREF_FLAG = 1 << 2;
  constexpr static uint8_t COPYABLE_FLAG = 1 << 3;
  constexpr static uint8_t MOVEABLE_FLAG = 1 << 4;

public:
  TypeProperties() = default;

  template <typename T>
  constexpr TypeProperties(Type<T> t)
    : _type(anyf::type_id(knot::decay(t)))
    , _category(knot::category(t))
  {
    if constexpr(anyf::is_const(t))
      _properties |= TypeProperties::CONST_FLAG;
    if constexpr(anyf::is_lref(t))
      _properties |= TypeProperties::LREF_FLAG;
    else if constexpr(anyf::is_rref(t))
      _properties |= TypeProperties::RREF_FLAG;
    if constexpr(std::is_copy_constructible_v<T>)
      _properties |= TypeProperties::COPYABLE_FLAG;
    if constexpr(std::is_move_constructible_v<T>)
      _properties |= TypeProperties::MOVEABLE_FLAG;
  }

  constexpr TypeID type_id() const { return _type; }

  constexpr bool is_const() const { return (_properties & CONST_FLAG) > 0; }
  constexpr bool is_cref() const { return is_const() && is_lref(); }
  constexpr bool is_lref() const { return (_properties & LREF_FLAG) > 0; }
  constexpr bool is_rref() const { return (_properties & RREF_FLAG) > 0; }
  constexpr bool is_ref() const { return is_lref() || is_rref(); }
  constexpr bool is_copy_constructible() const { return (_properties & COPYABLE_FLAG) > 0; }
  constexpr bool is_move_constructible() const { return (_properties & MOVEABLE_FLAG) > 0; }

  constexpr auto props() const { return _properties; }

  constexpr friend bool operator==(const TypeProperties& lhs, const TypeProperties& rhs) {
    return lhs.type_id() == rhs.type_id() && lhs._properties == rhs._properties;
  }

  constexpr friend bool operator!=(const TypeProperties& lhs, const TypeProperties& rhs) { return !(lhs == rhs); }

private:
  TypeID _type = 0;
  TypeCategory _category;
  uint8_t _properties = 0;
};

template <typename... Ts>
std::vector<TypeProperties> make_types(TypeList<Ts...>) {
  return {TypeProperties(Type<Ts>{})...};
}

} // namespace anyf
