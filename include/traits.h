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

template <typename T>
struct is_const_ref : std::false_type {};

template <typename T>
struct is_const_ref<T const&> : std::true_type {};

template <typename T>
constexpr bool is_const_ref_v = is_const_ref<T>::value;

template <typename T, typename Enable = void>
struct is_decayed_or_cref_impl : std::false_type {};

template <typename T>
struct is_decayed_or_cref_impl<
    T, std::enable_if_t<std::is_same_v<T, std::decay_t<T>>>> : std::true_type {
};

template <typename T>
struct is_decayed_or_cref_impl<
    T, std::enable_if_t<std::is_same_v<T, const std::decay_t<T>&>>>
    : std::true_type {};

template <typename T>
struct is_decayed_or_cref : is_decayed_or_cref_impl<T> {};

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

template <template <typename> typename, typename>
struct tuple_count;

template <template <typename> typename Pred, typename... Ts>
struct tuple_count<Pred, std::tuple<Ts...>> {
  static constexpr int value = ((Pred<Ts>::value ? 1 : 0) + ...);
};

template <template <typename> typename Pred, typename Tuple>
constexpr int tuple_count_v = tuple_count<Pred, Tuple>::value;

template <template <typename> typename, typename>
struct tuple_pred_array;

template <template <typename> typename Pred, typename... Ts>
struct tuple_pred_array<Pred, std::tuple<Ts...>> {
  static constexpr std::array<bool, sizeof...(Ts)> value = {Pred<Ts>::value...};
};

template <template <typename> typename Pred, typename Tuple>
constexpr auto tuple_pred_array_v = tuple_pred_array<Pred, Tuple>::value;

template <typename>
struct tuple_drop_first;

template <>
struct tuple_drop_first<std::tuple<>> {
  using type = std::tuple<>;
};

template <typename T, typename... Ts>
struct tuple_drop_first<std::tuple<T, Ts...>> {
  using type = std::tuple<Ts...>;
};

template <typename Tuple>
using tuple_drop_first_t = typename tuple_drop_first<Tuple>::type;

} // namespace traits
} // namespace anyf

#endif