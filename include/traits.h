#ifndef TRAITS_H
#define TRAITS_H

#include <functional>
#include <tuple>
#include <type_traits>

namespace anyf {
namespace traits {

template <typename>
struct function_traits;

template <typename Function>
struct function_traits
    : public function_traits<decltype(&Function::operator())> {};

template <typename Class, typename Ret, typename... Args>
struct function_traits<Ret (Class::*)(Args...) const> {
  static constexpr std::size_t arity = sizeof...(Args);

  using return_type = Ret;
  using args = std::tuple<Args...>;
};

template <typename Ret, typename... Args>
struct function_traits<Ret (*)(Args...)> {
  static constexpr std::size_t arity = sizeof...(Args);

  using return_type = Ret;
  using args = std::tuple<Args...>;
};

template <typename T, typename Enable = void>
struct is_decayed_impl : std::false_type {};

template <typename T>
struct is_decayed_impl<T, std::enable_if_t<std::is_same_v<T, std::decay_t<T>>>>
    : std::true_type {};

template <typename T>
struct is_decayed : is_decayed_impl<T> {};

template <typename T>
constexpr bool is_decayed_v = is_decayed<T>::value;

template <template <typename> typename, typename>
struct tuple_all_of;

template <template <typename> typename Pred, typename... Ts>
struct tuple_all_of<Pred, std::tuple<Ts...>> {
  static constexpr bool value = (Pred<Ts>::value && ...);
};

template <template <typename> typename Pred, typename Tuple>
constexpr bool tuple_all_of_v = tuple_all_of<Pred, Tuple>::value;

template <template <typename> typename, typename>
struct tuple_any_of;

template <template <typename> typename Pred, typename... Ts>
struct tuple_any_of<Pred, std::tuple<Ts...>> {
  static constexpr bool value = (Pred<Ts>::value || ...);
};

template <template <typename> typename Pred, typename Tuple>
constexpr bool tuple_any_of_v = tuple_any_of<Pred, Tuple>::value;

} // namespace traits
} // namespace anyf

#endif