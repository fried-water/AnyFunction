#ifndef any_function_H
#define any_function_H

#include "traits.h"

#include "util.h"

#include <boost/container/small_vector.hpp>

#include <any>
#include <functional>
#include <iostream>
#include <typeindex>

namespace anyf {

class bad_any_value_invocation : std::exception {
  const char* what() const noexcept override {
    return "bad any value invocation.";
  }
};

template <typename Output, typename... Inputs>
class function_graph;

class any_function {
public:
  using vec_val_type = small_vec3<std::any>;
  using vec_ptr_type = small_vec3<std::any*>;
  using func_type = std::function<std::any(vec_ptr_type)>;

  any_function
  decorate(std::function<func_type(func_type func)> decorator) const {
    return any_function(decorator(_func), _output_type, _input_types,
                        _input_by_cref);
  }

  std::any invoke(vec_ptr_type inputs) const {
    return _func(std::move(inputs));
  }

  const small_vec_base<std::type_index>& input_types() const {
    return _input_types;
  }
  std::type_index output_type() const { return _output_type; }

  const small_vec_base<bool>& input_by_cref() const { return _input_by_cref; }

private:
  func_type _func;

  small_vec3<std::type_index> _input_types;
  std::type_index _output_type;

  small_vec3<bool> _input_by_cref;

  explicit any_function(func_type func, std::type_index output_type,
                        small_vec3<std::type_index> input_types,
                        small_vec3<bool> input_by_cref)
      : _func(std::move(func)), _input_types(std::move(input_types)),
        _output_type(output_type), _input_by_cref(input_by_cref) {}

  template <typename F>
  friend any_function make_any_function(F f);
};

template <typename F>
any_function make_any_function(F f);

namespace details {

template <typename TupleTypes, std::size_t... Is>
small_vec<std::type_index, 3> make_types_impl(std::index_sequence<Is...>) {
  return util::make_vector<small_vec<std::type_index, 3>>(
      std::type_index(typeid(std::tuple_element_t<Is, TupleTypes>))...);
}

template <typename TupleTypes>
small_vec<std::type_index, 3> make_types() {
  return make_types_impl<TupleTypes>(
      std::make_index_sequence<std::tuple_size_v<TupleTypes>>());
}

template <typename Return, typename Types, typename F, std::size_t... Is>
Return call_with_any_vec(F f, any_function::vec_ptr_type inputs,
                         std::index_sequence<Is...>) {
  if constexpr(std::is_same_v<void, Return>) {
    f(std::any_cast<std::tuple_element_t<Is, Types>>(
        std::move(*inputs[Is]))...);
  } else {
    return f(std::any_cast<std::tuple_element_t<Is, Types>>(
        std::move(*inputs[Is]))...);
  }
}
} // namespace details

template <typename F>
any_function make_any_function(F f) {
  using f_traits = traits::function_traits<F>;
  using ret_type = typename f_traits::return_type;
  using args = typename f_traits::args;

  constexpr bool legal_return =
      std::is_same_v<void, ret_type> || traits::is_decayed_v<ret_type>;

  constexpr bool legal_args =
      traits::tuple_all_of_v<traits::is_decayed_or_cref, args>;

  if constexpr(legal_return && legal_args) {
    auto&& func = [f = std::move(f)](any_function::vec_ptr_type inputs) {
      if constexpr(std::is_same_v<void, ret_type>) {
        details::call_with_any_vec<ret_type, args>(
            std::move(f), std::move(inputs),
            std::make_index_sequence<f_traits::arity>());
        return std::any();
      } else {
        return std::any(details::call_with_any_vec<ret_type, args>(
            std::move(f), std::move(inputs),
            std::make_index_sequence<f_traits::arity>()));
      }
    };

    return any_function(std::move(func), std::type_index(typeid(ret_type)),
                        details::make_types<args>(),
                        util::tuple_type_extraction<std::is_reference, args,
                                                    small_vec<bool, 3>>());
  } else {
    static_assert(legal_return,
                  "Function return type must be a value or void.");
    static_assert(legal_args,
                  "Function arguments must either be values on const refs.");
    throw;
  }
}

} // namespace anyf

#endif