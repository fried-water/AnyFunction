#include "higher_order.h"

#include "sentinal.h"

#define BOOST_TEST_DYN_LINK
#include <boost/test/unit_test.hpp>

using namespace anyf;

int multiply(int x, const int& y) { return x * y; }

BOOST_AUTO_TEST_CASE(test_map) {
  std::vector<int> input{1, 2, 3};

  auto multiply_map = map(multiply);

  std::vector<int> expected_2{2, 4, 6};
  std::vector<int> expected_5{5, 10, 15};

  BOOST_CHECK(expected_2 == multiply_map(input, 2));
  BOOST_CHECK(expected_5 == multiply_map(input, 5));
}
