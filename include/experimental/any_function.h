#ifndef ANY_FUNCTION_H
#define ANY_FUNCTION_H

#include "traits.h"
#include "util.h"

#include <boost/container/small_vector.hpp>

#include <any>
#include <functional>
#include <typeindex>

namespace anyf {

class bad_any_invocation : std::exception {
  const char* what() const noexcept override { return "Dad any invocation."; }
};

class any_function {
public:
  using vec_type = small_vec3<std::any>;

  template <typename Output, typename... Input>
  Output invoke(Input&&... input) const {
    auto input_vec = util::make_vector<vec_type>(
        std::make_any<std::add_pointer_t<std::decay_t<Input>>>(
            const_cast<std::add_pointer_t<std::decay_t<Input>>>(&input))...);
    auto should_move = util::make_vector<small_vec<bool, 3>>(
        (std::is_rvalue_reference_v<Input&&> &&
         !std::is_const_v<std::remove_reference_t<Input>>)...);

    if constexpr(std::is_same_v<void, Output>) {
      _func(input_vec, should_move);
    } else if constexpr(std::is_reference_v<Output>) {
      return *std::any_cast<
          std::add_pointer_t<std::remove_reference_t<Output>>>(
          _func(input_vec, should_move));
    } else {
      return std::any_cast<Output>(_func(input_vec, should_move));
    }
  }

  const small_vec_base<std::type_index>& input_types() const {
    return _input_types;
  }
  std::type_index output_type() const { return _output_type; }

private:
  std::function<std::any(const small_vec3<std::any>&,
                         const small_vec<bool, 3>&)>
      _func;

  small_vec3<std::type_index> _input_types;
  std::type_index _output_type;

  explicit any_function(std::function<std::any(const small_vec3<std::any>&,
                                               const small_vec<bool, 3>&)>
                            func,
                        std::type_index output_type,
                        small_vec3<std::type_index> input_types)
      : _func(std::move(func)), _input_types(std::move(input_types)),
        _output_type(output_type) {}

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

template <typename T>
T transform_input(std::decay_t<T>* input, bool move) {
  if constexpr(std::is_rvalue_reference_v<T>) {
    if(!move)
      throw bad_any_invocation();
    return std::move(*input);
  } else if constexpr(std::is_lvalue_reference_v<T>) {
    if(move && !std::is_const_v<std::remove_reference_t<T>>)
      throw bad_any_invocation();
    return *input;
  } else {
    return move ? std::move(*input) : *input;
  }
}

template <typename Return, typename Types, typename F, std::size_t... Is>
Return call_with_any_pointers_vec(F f, const small_vec<std::any, 3>& inputs,
                                  const small_vec<bool, 3>& should_move,
                                  std::index_sequence<Is...>) {
  if constexpr(std::is_same_v<void, Return>) {
    f(transform_input<std::tuple_element_t<Is, Types>>(
        std::any_cast<
            std::add_pointer_t<std::decay_t<std::tuple_element_t<Is, Types>>>>(
            inputs[Is]),
        should_move[Is])...);
  } else {
    return f(transform_input<std::tuple_element_t<Is, Types>>(
        std::any_cast<
            std::add_pointer_t<std::decay_t<std::tuple_element_t<Is, Types>>>>(
            inputs[Is]),
        should_move[Is])...);
  }
}
} // namespace details

template <typename F>
any_function make_any_function(F f) {
  using f_traits = traits::function_traits<F>;
  using ret_type = typename f_traits::return_type;
  using args = typename f_traits::args;

  auto&& func = [f](const small_vec<std::any, 3>& inputs,
                    const small_vec<bool, 3>& should_move) {
    if constexpr(std::is_same_v<void, ret_type>) {
      details::call_with_any_pointers_vec<ret_type, args>(
          std::move(f), inputs, should_move,
          std::make_index_sequence<f_traits::arity>());
      return std::any();
    } else if constexpr(std::is_reference_v<ret_type>) {
      auto& result = details::call_with_any_pointers_vec<ret_type, args>(
          std::move(f), inputs, should_move,
          std::make_index_sequence<f_traits::arity>());
      return std::make_any<
          std::add_pointer_t<std::remove_reference_t<ret_type>>>(&result);
    } else {
      return std::any(details::call_with_any_pointers_vec<ret_type, args>(
          std::move(f), inputs, should_move,
          std::make_index_sequence<f_traits::arity>()));
    }
  };

  return any_function(std::move(func), std::type_index(typeid(ret_type)),
                      details::make_types<args>());
}

} // namespace anyf

#endif