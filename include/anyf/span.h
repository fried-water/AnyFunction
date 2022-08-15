#pragma once

#include <algorithm>
#include <optional>

namespace anyf {

template <typename T>
class Span {
  const T* _begin = nullptr;
  const T* _end = nullptr;

public:
  using value_type = T;
  using reference_type = const T&;
  using pointer_type = const T*;
  using iterator_type = const T*;
  using const_iterator_type = const T*;

  Span() = default;

  Span(const T* begin, const T* end) : _begin(begin), _end(end) {}
  Span(const T* begin, std::size_t count) : _begin(begin), _end(begin + count) {}

  template<typename Range, typename = std::enable_if_t<std::is_same_v<T, typename std::decay_t<Range>::value_type>>>
  Span(Range&& rng) : _begin(std::data(rng)), _end(std::data(rng) + rng.size()) {}

  Span(std::initializer_list<T> il) : _begin(std::data(il)), _end(std::data(il) + il.size()) {}

  Span(const std::optional<T>& opt)
      : _begin(opt ? std::addressof(*opt) : nullptr), _end(opt ? std::addressof(*opt) + 1 : nullptr) {}

  const T* begin() const { return _begin; }
  const T* end() const { return _end; }

  const T* cbegin() const { return _begin; }
  const T* cend() const { return _end; }

  const T& front() const { return *_begin; }
  const T& back() const { return *(_end - 1); }

  const T& operator[](size_t idx) const { return *(_begin + idx); }

  Span first(std::size_t count) const { return {_begin, count}; }
  Span last(std::size_t count) const { return {_end - count, _end}; }
  Span subspan(std::size_t offset, std::size_t count = -1) const {
    return {_begin + offset, count == -1 ? _end : _end + count};
  }

  size_t size() const { return std::distance(_begin, _end); }
  int64_t ssize() const { return static_cast<int64_t>(size()); }

  bool empty() const { return _begin == _end; }

  const T* data() const { return _begin; }

  friend bool operator==(const Span<T>& lhs, const Span<T>& rhs) {
    return std::equal(lhs.begin(), lhs.end(), rhs.begin(), rhs.end());
  }

  friend bool operator!=(const Span<T>& lhs, const Span<T>& rhs) { return !(lhs == rhs); }
};

} // namespace anyf
