#ifndef any_function_H
#define any_function_H

#include "traits.h"
#include "type.h"

#include "util.h"

#include <boost/container/small_vector.hpp>

#include <any>
#include <functional>
#include <iostream>

namespace anyf {

class BadInvocation : std::exception {
  const char* what() const noexcept override { return "bad invocation"; }
};

class AnyFunction {
public:
  using InvokeResult = SmallVec<std::any, 1>;
  using InvokeInput = SmallVec<std::any*, 3>;

  template <typename F>
  explicit AnyFunction(F f);

  InvokeResult invoke(InvokeInput inputs) const {
    if(inputs.size() != input_types().size())
      throw BadInvocation();
    return _func(std::move(inputs));
  }

  const SmallVecBase<Type>& input_types() const { return _input_types; }
  const SmallVecBase<Type>& output_types() const { return _output_types; }

private:
  std::function<InvokeResult(InvokeInput)> _func;

  SmallVec<Type, 3> _input_types;
  SmallVec<Type, 1> _output_types;
};

namespace details {
template <typename Types, typename F, std::size_t... Is>
AnyFunction::InvokeResult call_with_any_vec(F f, AnyFunction::InvokeInput inputs,
                                            std::index_sequence<Is...>) {
  using Ret = typename traits::function_traits<F>::return_type;

  if constexpr(std::is_same_v<void, Ret>) {
    f(std::any_cast<std::tuple_element_t<Is, Types>>(std::move(*inputs[Is]))...);
    return {};
  } else if constexpr(traits::is_tuple_v<Ret>) {
    Ret result_tuple = f(std::any_cast<std::tuple_element_t<Is, Types>>(std::move(*inputs[Is]))...);

    AnyFunction::InvokeResult result;
    result.reserve(std::tuple_size_v<Ret>);
    util::tuple_for_each(std::move(result_tuple),
                         [&result](int, auto x) { result.push_back(std::move(x)); });
    return result;
  } else {
    return util::make_vector<AnyFunction::InvokeResult>(
        f(std::any_cast<std::tuple_element_t<Is, Types>>(std::move(*inputs[Is]))...));
  }
}
} // namespace details

template <typename T>
constexpr bool valid_return_type() {
  if constexpr(std::is_same_v<void, T>)
    return true;
  else if constexpr(!traits::is_tuple_v<T>)
    return traits::is_decayed_v<T> && !std::is_pointer_v<T>;
  else
    return traits::tuple_all_of_v<traits::is_decayed, T> &&
           traits::tuple_none_of_v<std::is_pointer, T>;
}

template <typename F>
AnyFunction::AnyFunction(F f)
    : _input_types(make_types<decltype(_input_types), traits::function_args_t<F>>()),
      _output_types(make_types<decltype(_output_types),
                               traits::tuple_wrap_t<traits::function_return_t<F>>>()) {
  using f_traits = traits::function_traits<F>;
  using ret_type = traits::function_return_t<F>;
  using args = traits::function_args_t<F>;

  constexpr bool legal_args = traits::tuple_all_of_v<traits::is_decayed_or_cref, args> &&
                              traits::tuple_none_of_v<std::is_pointer, args>;

  if constexpr(valid_return_type<ret_type>() && legal_args && f_traits::is_const) {
    _func = [f = std::move(f)](AnyFunction::InvokeInput inputs) {
      return details::call_with_any_vec<args>(std::move(f), std::move(inputs),
                                              std::make_index_sequence<f_traits::arity>());
    };
  } else {
    static_assert(f_traits::is_const, "No mutable lambdas and non-const operator().");
    static_assert(valid_return_type<ret_type>(),
                  "Function return type must be void, a value or tuple of "
                  "values (no refs or pointers).");
    static_assert(legal_args, "Function arguments must either be values on "
                              "const refs (no non-const refs or pointers).");
    throw;
  }
}

} // namespace anyf

#endif