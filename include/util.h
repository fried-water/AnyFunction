#pragma once

#include <fmt/core.h>

#include <array>
#include <optional>
#include <tuple>
#include <utility>
#include <vector>

namespace anyf {

template <class... Ts>
struct Overloaded : Ts... {
  using Ts::operator()...;
};
template <class... Ts>
Overloaded(Ts...) -> Overloaded<Ts...>;

template <typename T, typename... Elements>
std::vector<T> make_vector(Elements&&... elements) {
  std::vector<T> vec;
  vec.reserve(sizeof...(Elements));
  (vec.emplace_back(std::forward<Elements>(elements)), ...);
  return vec;
}

template <typename... Ts>
decltype(auto) tuple_or_value(Ts&&... t) {
  if constexpr(sizeof...(Ts) == 1) {
    return std::forward<Ts...>(t...);
  } else {
    return std::tuple(std::forward<Ts>(t)...);
  }
}

namespace details {

template <typename Range, typename F, std::size_t... Is>
decltype(auto) apply_range(Range&& range, F&& f, std::index_sequence<Is...>) {
  if constexpr(std::is_reference_v<Range>) {
    return std::forward<F>(f)(range[Is]...);
  } else {
    return std::forward<F>(f)(std::move(range[Is])...);
  }
}

} // namespace details

template <size_t Count, typename Range, typename F>
decltype(auto) apply_range(Range&& range, F f) {
  return details::apply_range(std::forward<Range>(range), f, std::make_index_sequence<Count>());
}

template <typename F, typename... Inputs>
decltype(auto) invoke_normalize_void_return(F&& f, Inputs&&... inputs) {
  if constexpr(is_same(Ty<void>{}, return_type<std::decay_t<F>>())) {
    std::forward<F>(f)(std::forward<Inputs>(inputs)...);
    return std::tuple();
  } else {
    return std::forward<F>(f)(std::forward<Inputs>(inputs)...);
  }
}

template <typename... Ts>
[[noreturn]] void error(const char* fmt_string, const Ts&... ts) {
  fmt::print("Error: {}\n", fmt::format(fmt_string, ts...));
  std::abort();
}

template <typename... Ts>
void check(bool b, const char* fmt_string, const Ts&... ts) {
  if(!b) {
    error(fmt_string, ts...);
  }
}

template <typename T>
class Span {
  const T* _begin = nullptr;
  const T* _end = nullptr;

  using value_type = T;
  using reference_type = const T&;
  using pointer_type = const T*;
  using iterator_type = const T*;
  using const_iterator_type = const T*;

public:
  Span() = default;

  template <size_t N>
  Span(const std::array<T, N>& arr) : _begin(arr.data()), _end(arr.data() + arr.size()) {}

  Span(const std::vector<T>& vec) : _begin(vec.data()), _end(vec.data() + vec.size()) {}

  Span(const std::optional<T>& opt)
      : _begin(opt ? std::addressof(*opt) : nullptr), _end(opt ? std::addressof(*opt) + 1 : nullptr) {}

  const T* begin() const { return _begin; }
  const T* end() const { return _end; }

  const T* cbegin() const { return _begin; }
  const T* cend() const { return _end; }

  const T& front() const { return *_begin; }
  const T& back() const { return *(_end - 1); }

  const T& operator[](size_t idx) const { return *(_begin + idx); }

  size_t size() const { return std::distance(_begin, _end); }
  int64_t ssize() const { return static_cast<int64_t>(size()); }

  bool is_empty() const { return _begin == _end; }
};

} // namespace anyf
