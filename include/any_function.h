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

namespace anyf {

class any_function {
  constexpr static int SMALL_VEC_SIZE = 3;
 public:
  using vec_type = small_vec<std::any, SMALL_VEC_SIZE>;

  template <typename Output, typename... Input>
  Output invoke(Input&&... input) const {
    return std::any_cast<Output>(_func(util::make_vector<vec_type>(std::any(std::forward<Input>(input))...)));
  }

  std::any invoke_any(vec_type input) const {
    return _func(std::move(input));
  }

  const small_vec_base<Type>& input_types() const { return _input_types; }
  Type output_type() const { return _output_type; }

 private:
  std::function<std::any(vec_type)> _func;

  small_vec<Type, SMALL_VEC_SIZE> _input_types;
  Type _output_type;

  explicit any_function(std::function<std::any(vec_type)> func, small_vec<Type, SMALL_VEC_SIZE> input_types, Type output_type) :
      _func(std::move(func)),
      _input_types(std::move(input_types)),
      _output_type(output_type) { }

  template <typename F>
  friend any_function make_any_function(F f);
};

template <typename F>
inline any_function make_any_function(F f) {
  using f_traits = traits::function_traits<F>;
  using ret_type = typename f_traits::return_type;
  using args = typename f_traits::args;

  constexpr bool legal_return_type = std::is_same_v<std::decay_t<ret_type>, ret_type>;
  constexpr bool has_pointer = std::is_pointer_v<ret_type> || traits::tuple_any_of_v<std::is_pointer, args>;

  static_assert(legal_return_type && !has_pointer, "Arguments and return type cannot be pointers, and return type must decay to itself");

  if constexpr(legal_return_type && !has_pointer) {
    return any_function(
      [f = std::move(f)](small_vec<std::any, 3> inputs) {
        assert(inputs.size() == f_traits::arity);
        return std::any(util::call_with_any_vec<args>(std::move(f), std::move(inputs), std::make_index_sequence<f_traits::arity>()));
      },
      make_types<args>(),
      make_type<ret_type>());
  }
}

}

#endif