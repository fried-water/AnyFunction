#ifndef ANY_FUNCTION_H
#define ANY_FUNCTION_H

#include "util.h"
#include "type.h"
#include "traits.h"

#include <boost/container/small_vector.hpp>

#include <any>
#include <functional>

namespace anyf {

class any_function {
  constexpr static int SMALL_VEC_SIZE = 3;
 public:
  using vec_type = small_vec<std::any, SMALL_VEC_SIZE>;

  template <typename Output, typename... Input>
  Output invoke(Input&&... input) const {
    auto vec = util::make_vector<vec_type>(std::forward<Input>(input)...);
    return std::any_cast<Output>(_func(vec));
  }

  std::any invoke(small_vec<std::any, 3>& inputs) const {
    return _func(inputs);
  }

  const small_vec_base<Type>& input_types() const { return _input_types; }
  Type output_type() const { return _output_type; }

 private:
  std::function<std::any(vec_type&)> _func;

  small_vec<Type, SMALL_VEC_SIZE> _input_types;
  Type _output_type;

  explicit any_function(std::function<std::any(vec_type&)> func, small_vec<Type, SMALL_VEC_SIZE> input_types, Type output_type) :
      _func(std::move(func)),
      _input_types(std::move(input_types)),
      _output_type(output_type) { }

  template <typename F>
  friend any_function make_any_function(F f);
};

template <typename F>
inline any_function make_any_function(F f);

namespace details {
template <typename Types, typename Vec, typename F, std::size_t... Is>
inline auto call_with_any_vec(F f, Vec& inputs, std::index_sequence<Is...>) {
  return std::invoke(f, std::any_cast<std::tuple_element_t<Is, Types>>(std::move(inputs[Is]))...);
}
}

template <typename F>
inline any_function make_any_function(F f) {
  using namespace traits;
  using ret_type = typename function_traits<F>::return_type;
  using args = typename function_traits<F>::args;

  constexpr bool everything_decayed = tuple_all_of_v<is_decayed, args> && is_decayed_v<ret_type>;

  static_assert(everything_decayed, "Arguments and return types must be decayed.");

  if constexpr(everything_decayed) {
    return any_function(
      [f = std::move(f)](small_vec<std::any, 3>& inputs) {
        return std::any(details::call_with_any_vec<args>(std::move(f), inputs, std::make_index_sequence<function_traits<F>::arity>()));
      },
      make_types<args>(),
      make_type<ret_type>());
  } else {
    throw 0;
  }
}

}

#endif