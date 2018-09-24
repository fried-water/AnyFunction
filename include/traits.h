#ifndef TRAITS_H
#define TRAITS_H

#include <functional>
#include <tuple>
#include <type_traits>

namespace comp {
namespace traits {

template <typename>
struct function_traits;

template <typename Function>
struct function_traits : public function_traits<decltype(&Function::operator())> {};

template <typename Class, typename Ret, typename... Args>
struct function_traits<Ret(Class::*)(Args...) const> {
    static constexpr std::size_t arity = sizeof...(Args);

    using return_type = Ret;
    using args = std::tuple<Args...>;
};

template <typename Ret, typename... Args>
struct function_traits<Ret(*)(Args...)> {
    static constexpr std::size_t arity = sizeof...(Args);

    using return_type = Ret;
    using args = std::tuple<Args...>;
};

template <typename T>
struct is_tuple : std::false_type {};

template <typename... Ts>
struct is_tuple<std::tuple<Ts...>> : std::true_type {};


template <template <typename> typename, typename>
struct tuple_all_of;

template <template <typename> typename Pred, typename... Ts>
struct tuple_all_of<Pred, std::tuple<Ts...>> {
  static constexpr bool value = (Pred<Ts>::value && ...);
};

template <template <typename> typename Pred, typename Tuple>
constexpr bool tuple_all_of_v = tuple_all_of<Pred, Tuple>::value;

template <template <typename> typename,typename>
struct tuple_any_of;

template <template <typename> typename Pred, typename... Ts>
struct tuple_any_of<Pred, std::tuple<Ts...>> {
  static constexpr bool value = (Pred<Ts>::value || ...);
};

template <template <typename> typename Pred, typename Tuple>
constexpr bool tuple_any_of_v = tuple_any_of<Pred, Tuple>::value;

}
}

#endif