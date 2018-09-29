#ifndef UTIL_H
#define UTIL_H

#include <boost/container/small_vector.hpp>

#include <any>
#include <functional>
#include <tuple>
#include <vector>

namespace anyf {

template <typename T, std::size_t N>
using small_vec = boost::container::small_vector<T, N>;

template <typename T>
using small_vec_base = boost::container::small_vector_base<T>;

namespace util {

template <typename Vec, typename... Elements>
inline Vec make_vector(Elements&&... elements) {
  Vec vec;
  vec.reserve(sizeof...(Elements));
  (vec.emplace_back(std::forward<Elements>(elements)), ...);
  return vec;
}

template <typename T, typename... Elements>
inline std::vector<T> make_std_vector(Elements&&... elements) {
  std::vector<T> vec;
  vec.reserve(sizeof...(Elements));
  (vec.emplace_back(std::forward<Elements>(elements)), ...);
  return vec;
}

template <typename OutputContainer, typename InputContainer, typename Transform>
inline OutputContainer map(InputContainer&& input_container,
                           Transform transform) {
  OutputContainer out;
  out.reserve(input_container.size());
  if constexpr(std::is_rvalue_reference_v<InputContainer&&>) {
    for(auto& input : input_container) {
      out.emplace_back(transform(std::move(input)));
    }
  } else {
    for(const auto& input : input_container) {
      out.emplace_back(transform(input));
    }
  }
  return out;
}

} // namespace util
} // namespace anyf

#endif