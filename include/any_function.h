#ifndef ANY_FUNCTION_H
#define ANY_FUNCTION_H

#include "traits.h"
#include "type.h"
#include "util.h"

#include <boost/container/small_vector.hpp>

#include <any>
#include <functional>

namespace anyf {

class any_function {
  constexpr static int SMALL_VEC_SIZE = 3;

public:
  using vec_type = small_vec<std::any, SMALL_VEC_SIZE>;

  template <typename Output, typename... Input>
  Output invoke(Input &&... input) const {
    // static_assert(!(std::is_rvalue_reference_v<Input&&> || ...), "Invoke does
    // not support rvalue references.");
    if constexpr (std::is_reference_v<Output>) {
      return *std::any_cast<
          std::add_pointer_t<std::remove_reference_t<Output>>>(
          _func_with_pointers(util::make_vector<vec_type>(
              std::make_any<std::add_pointer_t<std::decay_t<Input>>>(
                  &input)...)));
    } else {
      return std::any_cast<Output>(
          _func_with_pointers(util::make_vector<vec_type>(
              std::make_any<std::add_pointer_t<std::decay_t<Input>>>(
                  &input)...)));
    }
  }

  std::any invoke_any(small_vec<std::any, 3> &&inputs) const {
    return _func_with_values(std::move(inputs));
  }

  const small_vec_base<Type> &input_types() const { return _input_types; }
  Type output_type() const { return _output_type; }

private:
  std::function<std::any(const vec_type &)> _func_with_pointers;
  std::function<std::any(vec_type &&)> _func_with_values;

  small_vec<Type, SMALL_VEC_SIZE> _input_types;
  Type _output_type;

  explicit any_function(
      std::function<std::any(const vec_type &)> func_with_pointers,
      std::function<std::any(vec_type &&)> func_with_values,
      small_vec<Type, SMALL_VEC_SIZE> input_types, Type output_type)
      : _func_with_pointers(std::move(func_with_pointers)),
        _func_with_values(std::move(func_with_values)),
        _input_types(std::move(input_types)), _output_type(output_type) {}

  template <typename F> friend any_function make_any_function(F f);
};

template <typename F> inline any_function make_any_function(F f);

namespace details {
template <typename Return, typename Types, typename F, std::size_t... Is>
inline Return call_with_any_pointers_vec(F f,
                                         const small_vec<std::any, 3> &inputs,
                                         std::index_sequence<Is...>) {
  return f(*std::any_cast<
           std::add_pointer_t<std::decay_t<std::tuple_element_t<Is, Types>>>>(
      inputs[Is])...);
}

template <typename Return, typename Types, typename F, std::size_t... Is>
inline Return call_with_any_values_vec(F f, small_vec<std::any, 3> &&inputs,
                                       std::index_sequence<Is...>) {
  return f(
      std::any_cast<std::tuple_element_t<Is, Types>>(std::move(inputs[Is]))...);
}
} // namespace details

template <typename F> inline any_function make_any_function(F f) {
  using f_traits = traits::function_traits<F>;
  using ret_type = typename f_traits::return_type;
  using args = typename f_traits::args;

  if constexpr (std::is_reference_v<ret_type>) {
    auto &&ptr_func = [f](const small_vec<std::any, 3> &inputs) {
      auto &result = details::call_with_any_pointers_vec<ret_type, args>(
          std::move(f), inputs, std::make_index_sequence<f_traits::arity>());
      return std::make_any<
          std::add_pointer_t<std::remove_reference_t<ret_type>>>(&result);
    };

    auto &&val_func = [f = std::move(f)](small_vec<std::any, 3> &&inputs) {
      auto &result = details::call_with_any_values_vec<ret_type, args>(
          std::move(f), std::move(inputs),
          std::make_index_sequence<f_traits::arity>());
      return std::make_any<
          std::add_pointer_t<std::remove_reference_t<ret_type>>>(&result);
    };

    return any_function(ptr_func, val_func, make_types<args>(),
                        make_type<ret_type>());
  } else {
    auto &&ptr_func = [f](const small_vec<std::any, 3> &inputs) {
      return std::any(details::call_with_any_pointers_vec<ret_type, args>(
          std::move(f), inputs, std::make_index_sequence<f_traits::arity>()));
    };

    auto &&val_func = [f = std::move(f)](small_vec<std::any, 3> &&inputs) {
      return std::any(details::call_with_any_values_vec<ret_type, args>(
          std::move(f), std::move(inputs),
          std::make_index_sequence<f_traits::arity>()));
    };

    return any_function(ptr_func, val_func, make_types<args>(),
                        make_type<ret_type>());
  }
}

} // namespace anyf

#endif