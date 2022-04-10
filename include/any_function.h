#pragma once

#include "traits.h"
#include "type.h"

#include "util.h"

#include <any>
#include <exception>
// #include <functional>
// #include <iostream>

namespace anyf {

class BadInvocation : std::exception {
  const char* what() const noexcept override { return "bad invocation"; }
};

class AnyFunction {
public:
  template <typename F>
  explicit AnyFunction(F f);

  std::vector<std::any> operator()(std::vector<std::any*> inputs) const {
    if(inputs.size() != input_types().size())
      throw BadInvocation();
    return _func(std::move(inputs));
  }

  const std::vector<Type>& input_types() const { return _input_types; }
  const std::vector<Type>& output_types() const { return _output_types; }

private:
  std::function<std::vector<std::any>(std::vector<std::any*>&&)> _func;

  std::vector<Type> _input_types;
  std::vector<Type> _output_types;
};

namespace details {
template <typename... Ts, typename F, std::size_t... Is>
std::vector<std::any> call_with_any_vec(TL<Ts...>, F f, std::vector<std::any*>&& inputs,
                                        std::index_sequence<Is...>) {
  constexpr auto fn_ret = return_type<F>();

  if constexpr(is_same(Ty<void>{}, fn_ret)) {
    f(std::any_cast<Ts>(std::move(*inputs[Is]))...);
    return {};
  } else if constexpr(is_tuple(fn_ret)) {
    return std::apply([](auto&&... e) { return make_vector<std::any>(std::move(e)...); },
                      f(std::any_cast<Ts>(std::move(*inputs[Is]))...));
  } else {
    return make_vector<std::any>(f(std::any_cast<Ts>(std::move(*inputs[Is]))...));
  }
}

} // namespace details

template <typename F>
AnyFunction::AnyFunction(F f)
    : _input_types(make_types(args<F>())), _output_types(make_types(as_tl(return_type<F>()))) {
  constexpr auto fn_ret = return_type<F>();
  constexpr auto fn_args = args<F>();

  constexpr bool legal_return = none(as_tl(fn_ret), [](auto t) { return is_pointer(t); }) &&
                                all(as_tl(fn_ret), [](auto t) { return is_decayed(t); });
  constexpr bool legal_args = none(fn_args, [](auto t) { return is_pointer(t); }) &&
                              all(fn_args, [](auto t) { return is_decayed(t) || is_const_ref(t); });

  if constexpr(legal_return && legal_args && is_const<F>()) {
    _func = [f = std::move(f), fn_args](std::vector<std::any*>&& inputs) {
      return details::call_with_any_vec(fn_args, std::move(f), std::move(inputs),
                                        std::make_index_sequence<size(fn_args)>());
    };
  } else {
    static_assert(is_const<F>(), "No mutable lambdas and non-const operator().");
    static_assert(legal_return, "Function return type must be void, a value or tuple of "
                                "values (no refs or pointers).");
    static_assert(legal_args, "Function arguments must either be values on "
                              "const refs (no non-const refs or pointers).");
    throw;
  }
}

} // namespace anyf
