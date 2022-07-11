#pragma once

#include "anyf/any.h"
#include "anyf/span.h"
#include "anyf/traits.h"
#include "anyf/type.h"
#include "anyf/util.h"

#include <exception>
#include <functional>

namespace anyf {

struct BadInvocation final : std::exception {
  const char* what() const noexcept override { return "bad invocation"; }
};

struct BadBind final : std::exception {
  const char* what() const noexcept override { return "bad bind"; }
};

class AnyFunction {
public:
  template <typename F>
  explicit AnyFunction(F f);

  explicit AnyFunction(TypeProperties, Any);

  std::vector<Any> operator()(Span<Any*> inputs) const { return _func(inputs); }

  const std::vector<TypeProperties>& input_types() const { return _input_types; }
  const std::vector<TypeProperties>& output_types() const { return _output_types; }

  AnyFunction bind(Any, int idx) const;

private:
  AnyFunction() = default;

  std::function<std::vector<Any>(Span<Any*>)> _func;

  std::vector<TypeProperties> _input_types;
  std::vector<TypeProperties> _output_types;
};

namespace details {
template <typename... Ts, typename F, std::size_t... Is>
std::vector<Any> call_with_anys(TypeList<Ts...>, F& f, Span<Any*> inputs, std::index_sequence<Is...>) {
  if(inputs.size() != sizeof...(Ts) || ((type_id(decay(Type<Ts>{})) != inputs[Is]->type()) || ...)) {
    throw BadInvocation();
  }
  auto&& result = invoke_normalize_void_return(f, std::move(*any_cast<std::decay_t<Ts>>(inputs[Is]))...);

  if constexpr(is_tuple(decay(Type<decltype(result)>{}))) {
    return std::apply([](auto&&... e) { return make_vector<Any>(std::move(e)...); }, std::move(result));
  } else {
    return make_vector<Any>(std::move(result));
  }
}

} // namespace details

template <typename Range>
inline std::vector<Any*> any_ptrs(Range&& anys) {
  std::vector<Any*> ptrs;
  ptrs.reserve(anys.size());
  std::transform(anys.begin(), anys.end(), std::back_inserter(ptrs), [](Any& a) { return &a; });
  return ptrs;
}

template <typename F>
AnyFunction::AnyFunction(F f)
    : _input_types(make_types(args(Type<F>{}))), _output_types(make_types(return_types(Type<F>{}))) {
  constexpr Type<F> f_type = {};
  constexpr auto fn_ret = return_types(f_type);
  constexpr auto fn_args = args(f_type);

  constexpr bool legal_return = none(fn_ret, [](auto t) { return knot::is_raw_pointer(decay(t)); }) &&
                                all(fn_ret, [](auto t) { return knot::is_decayed(t) || is_rref(t); });
  constexpr bool legal_args = none(fn_args, [](auto t) { return knot::is_raw_pointer(decay(t)); }) &&
                              all(fn_args, [](auto t) { return knot::is_decayed(t) || is_const_ref(t) || is_rref(t); });

  if constexpr(legal_return && legal_args && is_const_function(f_type)) {
    _func = [f = std::move(f), fn_args](Span<Any*> inputs) {
      return details::call_with_anys(fn_args, f, inputs, knot::idx_seq(fn_args));
    };
  } else {
    static_assert(is_const_function(f_type), "No mutable lambdas and non-const operator().");
    static_assert(legal_return,
                  "Function return type must be void, a value or tuple of "
                  "values (no refs or pointers).");
    static_assert(legal_args,
                  "Function arguments must either be values on "
                  "const refs (no non-const refs or pointers).");
    throw;
  }
}

inline AnyFunction::AnyFunction(TypeProperties props, Any any)
    : _func([a = std::move(any)](Span<Any*>) { return std::vector<Any>{a}; }), _output_types{props} {}

inline AnyFunction AnyFunction::bind(Any v, int idx) const {
  if(idx >= _input_types.size() || idx < 0 || _input_types[idx].type_id() != v.type()) {
    throw BadBind{};
  }

  AnyFunction anyf;

  anyf._func = [v = std::move(v), idx, f = _func](Span<Any*> span) mutable {
    std::vector<Any*> anys;
    anys.reserve(span.size() + 1);

    std::copy(span.begin(), span.begin() + idx, std::back_inserter(anys));
    anys.push_back(&v);
    std::copy(span.begin() + idx, span.end(), std::back_inserter(anys));

    return f(anys);
  };

  anyf._input_types = _input_types;
  anyf._output_types = _output_types;

  anyf._input_types.erase(anyf._input_types.begin() + idx);

  return anyf;
}

} // namespace anyf
