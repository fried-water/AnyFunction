#include "graph.h"

#include <algorithm>

#define BOOST_TEST_DYN_LINK
#include <boost/test/unit_test.hpp>

using namespace anyf;
using namespace anyf::graph;

int identity(int x) { return x; }
auto gen(int x) { return [x](){ return x;}; }
auto gen_2(int x, int y) { return [x, y](){ return std::make_tuple(x, y);}; }

auto sum(int x, int y) { return x + y; }

std::ostream& operator<<(std::ostream& os, const Edge e) { 
  return os << e.src.node_id << " " << e.src.arg_idx << " -> " << e.dst.node_id << " " << e.dst.arg_idx;
}

bool compare_nodes(const Node& expected, const Node& actual) {
  return expected.types() == actual.types() &&
    expected.name == actual.name &&
    expected.outputs == actual.outputs;
}

template <typename F>
void check_bad_construction(F f, std::string_view error) {
  bool thrown = false;

  try {
    f();
  } catch(const BadConstruction& ex) {
    thrown = true;
    BOOST_CHECK(error == ex.msg);
    std::cout << ex.msg << "\n";
  }

  BOOST_CHECK(thrown);
}

// BOOST_AUTO_TEST_CASE(simple_graph) {
//   auto g = make_graph<int>({"in"})
//     .add(identity, "identity", {".in"})
//     .add(gen(5), "gen", {})
//     .add(sum, "sum", {"identity", "gen"})
//     .output<int>({"sum"});

//   std::vector<Node> expected_nodes{
//     Node{{{make_type<int>()}}, "", {"in"}},
//     Node{{make_any_function(identity)}, "identity", {}},
//     Node{{make_any_function(gen(5))}, "gen", {}},
//     Node{{make_any_function(sum)}, "sum", {}},
//     Node{{{make_type<int>()}}}
//   };

//   expected_nodes[0].outputs = { {{0, 0}, {1, 0}, PassBy::copy} };
//   expected_nodes[1].outputs = { {{1, 0}, {3, 0}, PassBy::copy} };
//   expected_nodes[2].outputs = { {{2, 0}, {3, 1}, PassBy::copy} };
//   expected_nodes[3].outputs = { {{3, 0}, {4, 0}, PassBy::copy} };

//   BOOST_CHECK(std::equal(
//     expected_nodes.begin(), expected_nodes.end(), g.nodes().begin(), g.nodes().end(), compare_nodes));
// }

BOOST_AUTO_TEST_CASE(simple_graph_in_graph) {
  auto inner = make_graph<int>({"in"})
    .add(identity, "id", {".in"})
    .add(identity, "id2", {"id"})
    .output<int>({"id2"});

  auto outer = make_graph<int>({"in"})
    .add(identity, "id", {".in"})
    .add(inner, "inner", {"id"})
    .add(identity, "id2", {"inner"})
    .output<int>({"id2"});
  
  std::vector<Node> expected_nodes{
    Node{{{make_type<int>()}}, "", {"in"}},
    Node{{make_any_function(identity)}, "id", {}},
    Node{{make_any_function(identity)}, "inner.id", {}},
    Node{{make_any_function(identity)}, "inner.id2", {}},
    Node{{make_any_function(identity)}, "id2", {}},
    Node{{{make_type<int>()}}}
  };

  expected_nodes[0].outputs = { {{0, 0}, {1, 0}, PassBy::copy} };
  expected_nodes[1].outputs = { {{1, 0}, {2, 0}, PassBy::copy} };
  expected_nodes[2].outputs = { {{2, 0}, {3, 0}, PassBy::copy} };
  expected_nodes[3].outputs = { {{3, 0}, {4, 0}, PassBy::copy} };
  expected_nodes[4].outputs = { {{4, 0}, {5, 0}, PassBy::copy} };


  BOOST_CHECK(outer.nodes().size() == expected_nodes.size());
  BOOST_CHECK(compare_nodes(outer.nodes()[0], expected_nodes[0]));
  BOOST_CHECK(compare_nodes(outer.nodes()[1], expected_nodes[1]));
  BOOST_CHECK(compare_nodes(outer.nodes()[2], expected_nodes[2]));
  BOOST_CHECK(compare_nodes(outer.nodes()[3], expected_nodes[3]));
  BOOST_CHECK(compare_nodes(outer.nodes()[4], expected_nodes[4]));

  BOOST_CHECK(std::equal(
    expected_nodes.begin(), expected_nodes.end(), outer.nodes().begin(), outer.nodes().end(), compare_nodes));
}

BOOST_AUTO_TEST_CASE(simple_graph_in_graph2) {
  int element = 10;

  auto inner =  make_graph<int>({"size"})
    .add([element](int size) { return size + element; }, "get", {".size"})
    .output<int>({"get"});

  auto outer_t = make_graph<int>({"size"});
  
  std::cout << "abc" << std::endl;

  auto outer_t2 = outer_t.add(inner, "pipe1", {"size"});
  
  std::cout << "def" << std::endl;

  auto outer = outer_t2.output<int>({"pipe1"});
  
  std::vector<Node> expected_nodes{
    Node{{{make_type<int>()}}, "", {"size"}},
    Node{{make_any_function(identity)}, "pipe1.get", {}},
    Node{{{make_type<int>()}}}
  };

  expected_nodes[0].outputs = { {{0, 0}, {1, 0}, PassBy::copy} };
  expected_nodes[1].outputs = { {{1, 0}, {2, 0}, PassBy::copy} };
  expected_nodes[2].outputs = { {{2, 0}, {3, 0}, PassBy::copy} };
  expected_nodes[3].outputs = { {{3, 0}, {4, 0}, PassBy::copy} };
  expected_nodes[4].outputs = { {{4, 0}, {5, 0}, PassBy::copy} };


  BOOST_CHECK(outer.nodes().size() == expected_nodes.size());
  BOOST_CHECK(compare_nodes(outer.nodes()[0], expected_nodes[0]));
  BOOST_CHECK(compare_nodes(outer.nodes()[1], expected_nodes[1]));
  BOOST_CHECK(compare_nodes(outer.nodes()[2], expected_nodes[2]));
  BOOST_CHECK(compare_nodes(outer.nodes()[3], expected_nodes[3]));
  BOOST_CHECK(compare_nodes(outer.nodes()[4], expected_nodes[4]));

  BOOST_CHECK(std::equal(
    expected_nodes.begin(), expected_nodes.end(), outer.nodes().begin(), outer.nodes().end(), compare_nodes));
}

// BOOST_AUTO_TEST_CASE(multi_output) {
//   auto g = make_graph<>({})
//     .add(gen_2(5, 6), "gen", {}, {"o1", "o2"})
//     .add(sum, "sum", {"gen.o1", "gen.o2"})
//     .output<int>({"sum"});

//   std::vector<Node> expected_nodes{
//     Node{{{}}, "", {}},
//     Node{{make_any_function(gen_2(5, 6))}, "gen", {"o1", "o2"}},
//     Node{{make_any_function(sum)}, "sum", {}},
//     Node{{{make_type<int>()}}}
//   };

//   expected_nodes[0].outputs = { };
//   expected_nodes[1].outputs = { {{1, 0}, {2, 0}, PassBy::copy}, {{1, 1}, {2, 1}, PassBy::copy} };
//   expected_nodes[2].outputs = { {{2, 0}, {3, 0}, PassBy::copy} };

//   BOOST_CHECK(std::equal(
//     expected_nodes.begin(), expected_nodes.end(), g.nodes().begin(), g.nodes().end(), compare_nodes));
// }

// BOOST_AUTO_TEST_CASE(valid_name) {
//   BOOST_CHECK(is_valid_name("a"));
//   BOOST_CHECK(is_valid_name("abc"));
//   BOOST_CHECK(is_valid_name("abc_"));
//   BOOST_CHECK(is_valid_name("B____34_"));

//   BOOST_CHECK(!is_valid_name(""));
//   BOOST_CHECK(!is_valid_name("_"));
//   BOOST_CHECK(!is_valid_name("1"));
//   BOOST_CHECK(!is_valid_name("_abc"));
//   BOOST_CHECK(!is_valid_name("2abc"));
//   BOOST_CHECK(!is_valid_name("^abc"));
// }

// BOOST_AUTO_TEST_CASE(invalid_port_name) {
//   check_bad_construction([](){ make_graph<int>({"_"}); }, "Invalid input: _");
//   check_bad_construction([](){ make_graph<int, std::string>({"a", "_"}); }, "Invalid input: _");
//   check_bad_construction([](){ make_graph<int, std::string>({"_1", "_2"}); }, "Invalid input: _1");

//   check_bad_construction([](){ make_graph<>({}).add(gen(5), "g", {}, {"_"}); }, "Invalid output: _");
//   check_bad_construction([](){ make_graph<>({}).add(gen_2(5, 6), "g", {}, {"a", "_"}); }, "Invalid output: _");
//   check_bad_construction([](){ make_graph<>({}).add(gen_2(5, 6), "g", {}, {"_1", "_2"}); }, "Invalid output: _1");
// }

// BOOST_AUTO_TEST_CASE(duplicate_port_name) {
//   check_bad_construction([](){ make_graph<int, std::string>({"a", "a"}); }, "Duplicate name: a");
//   check_bad_construction([](){ make_graph<>({}).add(gen_2(5, 6), "g", {}, {"o", "o"}); }, "Duplicate name: o");
// }

