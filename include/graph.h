#ifndef GRAPH_H
#define GRAPH_H

#include "any_function.h"
#include "type.h"
#include "util.h"

#include <variant>

#include <boost/container/small_vector.hpp>

namespace anyf {

struct Source {
  Type type;
  Source(Type type) : type(type) {}
};

struct Sink {
  int node_idx;
  Sink(int node_idx) : node_idx(node_idx) {}
};

struct InternalNode {
  any_function func;
  small_vec<int, 3> inputs;

  InternalNode(any_function func, small_vec<int, 3> inputs)
      : func(std::move(func)), inputs(std::move(inputs)) {}
};

template <typename... Inputs>
class constructing_graph;

template <typename Output, typename... Inputs>
class function_graph {
public:
  using NodeVariant = std::variant<Source, InternalNode, Sink>;

  friend class constructing_graph<Inputs...>;

  const std::vector<NodeVariant>& nodes() const { return _nodes; }
  const small_vec_base<std::pair<int, int>>& outputs(int i) const {
    return _outputs[i];
  }

private:
  function_graph(std::vector<NodeVariant> nodes,
                 std::vector<small_vec<std::pair<int, int>, 3>> outputs)
      : _nodes(std::move(nodes)), _outputs(std::move(outputs)) {}

  std::vector<NodeVariant> _nodes;
  // node -> id, idx
  std::vector<small_vec<std::pair<int, int>, 3>> _outputs;
};

template <typename... Inputs>
class constructing_graph {
  using NodeVariant = std::variant<Source, InternalNode, Sink>;
  std::vector<NodeVariant> _nodes;
  std::vector<std::string> _names;
  std::vector<small_vec<std::pair<int, int>, 3>> _outputs;

  constructing_graph(std::array<std::string, sizeof...(Inputs)> names)
      : _nodes(
            util::make_std_vector<NodeVariant>(Source(make_type<Inputs>())...)),
        _names(), _outputs(sizeof...(Inputs)) {
    _names.reserve(sizeof...(Inputs));
    ;
    for(auto&& name : std::move(names)) {
      assert(!name_exists(name));
      _names.push_back(name);
    }
  }

  Type get_type(int i) const {
    return std::visit(
        [this](const auto& arg) {
          using T = std::decay_t<decltype(arg)>;
          if constexpr(std::is_same_v<T, Source>) {
            return arg.type;
          } else if constexpr(std::is_same_v<T, Sink>) {
            return get_type(arg.node_idx);
          } else {
            return arg.func.output_type();
          }
        },
        _nodes[i]);
  }

  bool name_exists(const std::string_view& new_name) const {
    return std::find(_names.begin(), _names.end(), new_name) != _names.end();
  }

  int idx_of(const std::string_view& name) const {
    auto it = std::find(_names.begin(), _names.end(), name);
    assert(it != _names.end());

    return std::distance(_names.begin(), it);
  }

public:
  template <typename F>
  constructing_graph& add(F f, std::string name) {
    return add(std::move(f), std::move(name), {});
  }

  template <typename F>
  constructing_graph& add(F f, std::string name,
                          small_vec<std::string, 3> inputs) {
    assert(!name_exists(name));

    auto node = InternalNode(make_any_function(f),
                             util::map<small_vec<int, 3>>(
                                 inputs, [this](const std::string_view& name) {
                                   return idx_of(name);
                                 }));

    for(int i = 0; i < static_cast<int>(node.inputs.size()); i++) {
      Type output_type = get_type(node.inputs[i]);
      Type input_type = node.func.input_types()[i];

      if(output_type != input_type) {
        std::cout << "Output : " << output_type << " does not match "
                  << input_type << std::endl;
        throw 0;
      }

      _outputs[node.inputs[i]].emplace_back(_nodes.size(), i);
    }

    _nodes.push_back(std::move(node));
    _names.push_back(std::move(name));
    _outputs.emplace_back();
    return *this;
  }

  template <typename Output>
  function_graph<Output, Inputs...> output(const std::string_view& name) {
    int idx = idx_of(name);
    _outputs[idx].emplace_back(_nodes.size(), 0);
    _nodes.emplace_back(Sink(idx));
    _outputs.emplace_back();

    return function_graph<Output, Inputs...>(std::move(_nodes),
                                             std::move(_outputs));
  }

  template <typename... Ts>
  friend constructing_graph<Ts...>
  make_graph(std::array<std::string, sizeof...(Ts)> names);

  friend constructing_graph<> make_graph();
};

template <typename... Inputs>
inline constructing_graph<Inputs...>
make_graph(std::array<std::string, sizeof...(Inputs)> names) {
  return constructing_graph<Inputs...>(std::move(names));
}

inline constructing_graph<> make_graph() { return constructing_graph<>({}); }

} // namespace anyf

#endif