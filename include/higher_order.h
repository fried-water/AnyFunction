#ifndef HIGHER_ORDER_H
#define HIGHER_ORDER_H

#include "traits.h"

#include <functional>
#include <tuple>
#include <type_traits>

#include <iostream>

namespace anyf {

// Map
// For each
// Reduce
// Select
// While

namespace details {
template <typename F, typename Ret, typename TupleArgs>
struct map_helper;

template <typename F, typename Ret, typename T, typename... Args>
struct map_helper<F, Ret, std::tuple<T, Args...>> {
  F f;

  map_helper(F f) : f(std::move(f)) {}

  std::vector<Ret> operator()(std::vector<T> inputs, Args... args) {
    std::vector<Ret> results;
    results.reserve(inputs.size());
    for(T& t : inputs) {
      results.push_back(f(std::move(t), args...));
    };
    return results;
  }
};
} // namespace details

template <typename F>
auto map(F f) {
  using namespace traits;
  using Ret = typename function_traits<F>::return_type;
  using Args = typename function_traits<F>::args;

  constexpr int NUM_ARGS = std::tuple_size_v<Args>;

  static_assert(NUM_ARGS > 0, "Map requires function takes atleast one argument");

  if constexpr(NUM_ARGS > 0) {
    using first_arg = std::tuple_element_t<0, Args>;
    using rest_of_Args = tuple_drop_first_t<Args>;

    static_assert(is_decayed_v<first_arg>, "First argument must be taken by value");
    static_assert(tuple_all_of_v<is_const_ref, rest_of_Args>,
                  "Arguments beyond the first must be taken by const&");

    if constexpr(is_decayed_v<first_arg> && tuple_all_of_v<is_const_ref, rest_of_Args>) {
      return details::map_helper<F, Ret, Args>(std::move(f));
    }
  }

  throw;
}

} // namespace anyf

#endif