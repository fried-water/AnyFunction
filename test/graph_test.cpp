#include "graph.h"

#include <algorithm>

#define BOOST_TEST_DYN_LINK
#include <boost/test/unit_test.hpp>

using namespace anyf;

int identity(int x) { return x; }

BOOST_AUTO_TEST_CASE(simple_graph) {
  auto g = make_graph<int>({"in"})
               .add(identity, "identity", {".in"})
               .output<int>("identity");

  std::vector<std::string> expected_names{".in", "identity"};
  BOOST_TEST(expected_names == g.names());

  // auto outputs0 = std::vector<graph::edge>{graph::edge{1, 0, pass_by::move}};
  // BOOST_TEST(std::equal(outputs0.begin(), outputs0.end(),
  // g.outputs(0).begin()));

  // auto outputs1 = std::vector<graph::edge>{graph::edge{2, 0, pass_by::move}};
  // BOOST_TEST(std::equal(outputs1.begin(), outputs1.end(),
  // g.outputs(1).begin()));
}
