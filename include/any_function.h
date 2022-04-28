#pragma once

#include "span.h"
#include "traits.h"
#include "type.h"
#include "util.h"

#include <any>
#include <exception>
#include <functional>

namespace anyf {

struct BadInvocation : std::exception {
  const char* what() const noexcept override { return "bad invocation"; }
};

class AnyFunction {
public:
  template <typename F>
  explicit AnyFunction(F f);

  std::vector<std::any> operator()(Span<std::any*> inputs) const {
    if(inputs.size() != input_types().size())
      throw BadInvocation();
    return _func(inputs);
  }

  const std::vector<Type>& input_types() const { return _input_types; }
  const std::vector<Type>& output_types() const { return _output_types; }

private:
  std::function<std::vector<std::any>(Span<std::any*>)> _func;

  std::vector<Type> _input_types;
  std::vector<Type> _output_types;
};

namespace details {
template <typename... Ts, typename F, std::size_t... Is>
std::vector<std::any> call_with_anys(TL<Ts...>, F& f, Span<std::any*> inputs, std::index_sequence<Is...>) {
  if(((typeid(std::decay_t<Ts>) != inputs[Is]->type()) || ...)) {
    throw BadInvocation();
  }
  auto&& result = invoke_normalize_void_return(f, std::move(*std::any_cast<std::decay_t<Ts>>(inputs[Is]))...);

  if constexpr(is_tuple(decay(Ty<decltype(result)>{}))) {
    return std::apply([](auto&&... e) { return make_vector<std::any>(std::move(e)...); }, std::move(result));
  } else {
    return make_vector<std::any>(std::move(result));
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
    _func = [f = std::move(f), fn_args](Span<std::any*> inputs) {
      return details::call_with_anys(fn_args, f, inputs, std::make_index_sequence<size(fn_args)>());
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
