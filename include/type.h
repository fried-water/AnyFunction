#ifndef TYPE_H
#define TYPE_H

#include <array>
#include <iostream>
#include <sstream>
#include <string>

#include <tuple>
#include <type_traits>
#include <typeinfo>

#include <boost/core/demangle.hpp>

namespace anyf {

class Type;

std::ostream& operator<<(std::ostream& os, Type const& type);

class Type {
  constexpr static uint8_t CONST_FLAG = 1 << 0;
  constexpr static uint8_t REF_FLAG = 1 << 1;
  constexpr static uint8_t COPY_FLAG = 1 << 2;
  constexpr static uint8_t MOVE_FLAG = 1 << 3;

public:
  constexpr Type() : _type(&typeid(void)), _properties(0) {}

  constexpr bool is_const() const { return (_properties & CONST_FLAG) > 0; }
  constexpr bool is_ref() const { return (_properties & REF_FLAG) > 0; }
  constexpr bool is_copy_constructible() const { return (_properties & COPY_FLAG) > 0; }
  constexpr bool is_move_constructible() const { return (_properties & MOVE_FLAG) > 0; }

  std::type_info const& underlying_type() const { return *_type; }

  std::string name() const {
    std::ostringstream ostr;
    ostr << *this;

    // One day this will help
    return std::move(ostr).str();
  }

private:
  const std::type_info* _type;
  uint8_t _properties;

  template <typename T>
  friend constexpr Type make_type();

  explicit constexpr Type(std::type_info const* type, uint8_t properties)
      : _type(type), _properties(properties) {}
};

inline std::ostream& operator<<(std::ostream& os, Type const& type) {
  if(type.is_ref()) {
    os << boost::core::demangle(type.underlying_type().name());

    if(type.is_const())
      os << " const";

    os << "&";
  } else if(type.is_const()) {
    os << "const " << boost::core::demangle(type.underlying_type().name());
  } else {
    os << boost::core::demangle(type.underlying_type().name());
  }

  return os;
}

inline bool decayed_eq(Type const& x, Type const& y) {
  return x.underlying_type() == y.underlying_type();
}

inline bool operator==(Type const& x, Type const& y) {
  return decayed_eq(x, y) && x.is_const() == y.is_const() && x.is_ref() == y.is_ref();
}

inline bool operator!=(Type const& x, Type const& y) { return !(x == y); }

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

  return Type(&typeid(T), properties);
}

namespace detail {
template <typename T, typename Container>
struct TypeUnwrapper;

template <typename... Types, typename Container>
struct TypeUnwrapper<std::tuple<Types...>, Container> {
  constexpr Container operator()() const { return {make_type<Types>()...}; }
};
} // namespace detail

template <typename Tuple, typename Container = std::array<Type, std::tuple_size_v<Tuple>>>
Container make_types() {
  return detail::TypeUnwrapper<Tuple, Container>{}();
}

} // namespace anyf

#endif