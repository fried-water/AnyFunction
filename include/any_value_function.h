#ifndef ANY_VALUE_FUNCTION_H
#define ANY_VALUE_FUNCTION_H

#include "traits.h"

#include "util.h"

#include <boost/container/small_vector.hpp>

#include <any>
#include <functional>
#include <typeindex>

namespace anyf {

class bad_any_value_invocation : std::exception {
  const char* what() const noexcept override {
    return "bad any value invocation.";
  }
};

template <typename Output, typename... Inputs>
class function_graph;

class any_value_function {
  constexpr static int SMALL_VEC_SIZE = 3;

public:
  using vec_type = small_vec<std::any, SMALL_VEC_SIZE>;

  std::any invoke(small_vec<std::any, 3>&& inputs) const {
    return _func(std::move(inputs));
  }

  const small_vec_base<std::type_index>& input_types() const {
    return _input_types;
  }
  std::type_index output_type() const { return _output_type; }

private:
  std::function<std::any(vec_type&&)> _func;

  small_vec<std::type_index, SMALL_VEC_SIZE> _input_types;
  std::type_index _output_type;

  explicit any_value_function(
      std::function<std::any(vec_type&&)> func, std::type_index output_type,
      small_vec<std::type_index, SMALL_VEC_SIZE> input_types)
      : _func(std::move(func)), _input_types(std::move(input_types)),
        _output_type(output_type) {}

  template <typename F>
  friend any_value_function make_any_value_function(F f);

  template <typename Output, typename... Inputs>
  friend class function_graph;
};

template <typename F>
any_value_function make_any_value_function(F f);

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
Return call_with_any_values_vec(F f, small_vec<std::any, 3>&& inputs,
                                std::index_sequence<Is...>) {
  if constexpr(std::is_same_v<void, Return>) {
    f(std::any_cast<std::tuple_element_t<Is, Types>>(std::move(inputs[Is]))...);
  } else {
    return f(std::any_cast<std::tuple_element_t<Is, Types>>(
        std::move(inputs[Is]))...);
  }
}
} // namespace details

template <typename F>
any_value_function make_any_value_function(F f) {
  using f_traits = traits::function_traits<F>;
  using ret_type = typename f_traits::return_type;
  using args = typename f_traits::args;

  constexpr bool legal_return =
      std::is_same_v<void, ret_type> || traits::is_decayed_v<ret_type>;

  constexpr bool legal_args = traits::tuple_all_of_v<traits::is_decayed, args>;

  if constexpr(legal_return && legal_args) {
    auto&& func = [f = std::move(f)](small_vec<std::any, 3>&& inputs) {
      if constexpr(std::is_same_v<void, ret_type>) {
        details::call_with_any_values_vec<ret_type, args>(
            std::move(f), std::move(inputs),
            std::make_index_sequence<f_traits::arity>());
        return std::any();
      } else {
        return std::any(details::call_with_any_values_vec<ret_type, args>(
            std::move(f), std::move(inputs),
            std::make_index_sequence<f_traits::arity>()));
      }
    };

    return any_value_function(std::move(func),
                              std::type_index(typeid(ret_type)),
                              details::make_types<args>());
  } else {
    static_assert(legal_return, "Return type must be decayed or void");
    static_assert(legal_args,
                  "Args must be all be values (no const refs, refs, etc)");
    throw;
  }
}

} // namespace anyf

#endif