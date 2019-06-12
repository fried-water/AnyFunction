#ifndef TRAITS_H
#define TRAITS_H

#include <functional>
#include <tuple>
#include <type_traits>

namespace anyf {
namespace traits {

template <typename T>
using is_decayed =
    std::conditional_t<std::is_same_v<T, std::decay_t<T>>, std::true_type, std::false_type>;

template <typename T>
constexpr bool is_decayed_v = is_decayed<T>::value;

template <typename T>
using is_const_ref =
    std::conditional_t<std::is_same_v<T, const std::decay_t<T>&>, std::true_type, std::false_type>;

template <typename T>
constexpr bool is_const_ref_v = is_const_ref<T>::value;

template <typename T>
using is_decayed_or_cref =
    std::conditional_t<is_decayed_v<T> || is_const_ref_v<T>, std::true_type, std::false_type>;

template <typename T>
struct is_tuple : std::false_type {};

template <typename... Ts>
struct is_tuple<std::tuple<Ts...>> : std::true_type {};

template <typename T>
constexpr bool is_tuple_v = is_tuple<T>::value;

template<typename... Ts>
using first_t = std::tuple_element_t<0, std::tuple<Ts...>>;

template <template <typename> typename, typename>
struct tuple_all_of;

template <template <typename> typename Pred, typename... Ts>
struct tuple_all_of<Pred, std::tuple<Ts...>> {
  static constexpr bool value = (Pred<Ts>::value && ...);
};

template <template <typename> typename Pred, typename Tuple>
constexpr bool tuple_all_of_v = tuple_all_of<Pred, Tuple>::value;

template <template <typename> typename, typename>
struct tuple_none_of;

template <template <typename> typename Pred, typename... Ts>
struct tuple_none_of<Pred, std::tuple<Ts...>> {
  static constexpr bool value = (!Pred<Ts>::value && ...);
};

template <template <typename> typename Pred, typename Tuple>
constexpr bool tuple_none_of_v = tuple_none_of<Pred, Tuple>::value;

template <template <typename> typename, typename>
struct tuple_any_of;

template <template <typename> typename Pred, typename... Ts>
struct tuple_any_of<Pred, std::tuple<Ts...>> {
  static constexpr bool value = (Pred<Ts>::value || ...);
};

template <template <typename> typename Pred, typename Tuple>
constexpr bool tuple_any_of_v = tuple_any_of<Pred, Tuple>::value;

template <template <typename> typename, typename>
struct tuple_map;

template <template <typename> typename Pred, typename... Ts>
struct tuple_map<Pred, std::tuple<Ts...>> {
  using type = std::tuple<typename Pred<Ts>::type...>;
};

template <template <typename> typename Pred, typename Tuple>
using tuple_map_t = typename tuple_map<Pred, Tuple>::type;

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

// If T is a tuple, T, if T is void std::tuple<>, otherwise std::tuple<T>
template <typename T>
using tuple_wrap_t =
    std::conditional_t<is_tuple_v<T>, T,
                       std::conditional_t<std::is_same_v<T, void>, std::tuple<>, std::tuple<T>>>;

template <typename>
struct function_traits_impl;

template <typename Function>
struct function_traits_impl : public function_traits_impl<decltype(&Function::operator())> {};

template <typename Class, typename Ret, typename... Args>
struct function_traits_impl<Ret (Class::*)(Args...) const> {
  static constexpr bool is_const = true;

  using return_type = Ret;
  using args = std::tuple<Args...>;
};

template <typename Class, typename Ret, typename... Args>
struct function_traits_impl<Ret (Class::*)(Args...)> {
  static constexpr bool is_const = false;

  using return_type = Ret;
  using args = std::tuple<Args...>;
};

template <typename Ret, typename... Args>
struct function_traits_impl<Ret (*)(Args...)> {
  static constexpr bool is_const = true;

  using return_type = Ret;
  using args = std::tuple<Args...>;
};

template <typename Function>
struct function_traits {
  using return_type = typename function_traits_impl<Function>::return_type;
  using args = typename function_traits_impl<Function>::args;

  static constexpr std::size_t arity = std::tuple_size_v<args>;
  static constexpr std::size_t num_outputs = std::tuple_size_v<tuple_wrap_t<return_type>>;
  static constexpr bool is_const = function_traits_impl<Function>::is_const;
};

template <typename F>
using function_return_t = typename function_traits<F>::return_type;

template <typename F>
using function_args_t = typename function_traits<F>::args;

template <typename F>
constexpr std::size_t function_num_outputs_v = function_traits<F>::num_outputs;

template <typename F>
constexpr std::size_t function_num_inputs_v = function_traits<F>::arity;

} // namespace traits
} // namespace anyf

#endif