#ifndef GRAPH_H
#define GRAPH_H

#include "any_function.h"
#include "type.h"
#include "util.h"

#include <variant>

#include <boost/container/small_vector.hpp>


namespace comp {

struct Source {
  Type type;
  Source(Type type) :
    type(type) {}
};

struct Sink {
  int node_idx;
  Sink(int node_idx) :
    node_idx(node_idx) {}
};

struct InternalNode {
  AnyFunction func;
  small_vec<int, 3> inputs;
  
  InternalNode(AnyFunction func, small_vec<int, 3> inputs) :
    func(std::move(func)),
    inputs(std::move(inputs)) {}
};



class DependencyGraph {
  using NodeVariant = std::variant<Source, InternalNode, Sink>;
  std::vector<NodeVariant> _nodes;
  // node -> id, idx
  std::vector<small_vec<std::pair<int, int>, 3>> _outputs;
};

class NodeID {
  friend class ConstructingGraph;
  int idx;
  NodeID(int idx) : idx(idx) {}
};

class ConstructingGraph {
  using NodeVariant = std::variant<Source, InternalNode, Sink>;
  std::vector<NodeVariant> _nodes;
  std::vector<small_vec<std::pair<int, int>, 3>> _outputs;

  Type get_type(int i) {
    return std::visit([this](const auto& arg) {
      using T = std::decay_t<decltype(arg)>;
      if constexpr (std::is_same_v<T, Source>) {
        return arg.type;
      } else if constexpr (std::is_same_v<T, Sink>) {
        return get_type(arg.node_idx);
      } else {
        return arg.func.output_type();
      }
    }, _nodes[i]);
  }

 public:
  template <typename T>
  NodeID input() {
    _nodes.emplace_back(Source(make_type<T>()));
    _outputs.emplace_back();
    return _nodes.size() - 1;
  }

  template <typename F, typename... Nids>
  NodeID add(F f, Nids... ids) {
    static_assert((std::is_same_v<Nids, NodeID> && ...));
    auto node = InternalNode(make_any_function(f), util::create_small_vector<int, 3>(ids.idx...));

    for(int i = 0; i < static_cast<int>(node.inputs.size()); i++) {
      Type output_type = get_type(node.inputs[i]);
      Type input_type = node.func.input_types()[i];

      if(output_type != input_type) {
        std::cout << "Output : " << output_type << " does not match " << input_type << std::endl;
        throw 0;
      }

      _outputs[node.inputs[i]].emplace_back(_nodes.size(), i);
    }

    _nodes.emplace_back(std::move(node));
    _outputs.emplace_back();
    return _nodes.size() - 1;
  }

  DependencyGraph output(NodeID id) {
    _nodes.emplace_back(Sink(id.idx));
    return DependencyGraph();
  }
};

}

#endif