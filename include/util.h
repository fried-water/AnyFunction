#ifndef UTIL_H
#define UTIL_H

#include <boost/container/small_vector.hpp>

#include <any>
#include <tuple>
#include <functional>

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

}
}

#endif