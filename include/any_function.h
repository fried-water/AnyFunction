#ifndef ANY_FUNCTION_H
#define ANY_FUNCTION_H

#include "util.h"
#include "type.h"
#include "traits.h"

#include <boost/container/small_vector.hpp>

#include <cassert>

#include <any>
#include <iostream>
#include <functional>

namespace comp {

namespace details {

template <typename Types, typename F, std::size_t... Is>
inline auto call_with_vec(F f, small_vec<std::any, 3> inputs, std::index_sequence<Is...>) {
  return f(std::any_cast<std::tuple_element_t<Is, Types>>(std::move(inputs[Is]))...);
}

}

class AnyFunction {
    std::function<std::any(small_vec<std::any, 3>)> _func;

    small_vec<Type, 3> _input_types;
    Type _output_type;

  public:
    AnyFunction(std::function<std::any(small_vec<std::any, 3>)> func, small_vec<Type, 3> input_types, Type output_type) :
      _func(std::move(func)),
      _input_types(std::move(input_types)),
      _output_type(output_type) { }

    std::any run(small_vec<std::any, 3> inputs) const {
      return _func(std::move(inputs));
    }

    const small_vec_base<Type>& input_types() const { return _input_types; }
    Type output_type() const { return _output_type; }
};

template <typename F>
inline AnyFunction make_any_function(F f) {
  using f_traits = traits::function_traits<F>;
  using ret_type = typename f_traits::return_type;
  using args = typename f_traits::args;

  constexpr bool legal_return_type = std::is_same_v<std::decay_t<ret_type>, ret_type>;
  constexpr bool has_pointer = std::is_pointer_v<ret_type> || traits::tuple_any_of_v<std::is_pointer, args>;

  static_assert(legal_return_type && !has_pointer, "Arguments and return type cannot be pointers, and return type must decay to itself");

  if constexpr(legal_return_type && !has_pointer) {
    return AnyFunction(
      [f = std::move(f)](small_vec<std::any, 3> inputs) {
        assert(inputs.size() == f_traits::arity);
        return std::any(details::call_with_vec<args>(std::move(f), std::move(inputs), std::make_index_sequence<f_traits::arity>()));
      },
      make_types<args>(),
      make_type<ret_type>());
  }
}

template <typename Output, typename... Input>
inline Output invoke(const AnyFunction& func, Input&&... input) {
  return std::any_cast<Output>(func.run(util::create_small_vector<std::any, 3>(std::any(std::forward<Input>(input))...)));
}

}

#endif