#ifndef GRAPH_H
#define GRAPH_H

#include "any_function.h"
#include "type.h"
#include "util.h"

#include <numeric>
#include <variant>

#include <boost/container/small_vector.hpp>
#include <boost/range/algorithm.hpp>
#include <boost/range/combine.hpp>

namespace anyf {

template <typename... Inputs>
class constructing_graph;

template <typename Output, typename... Inputs>
class function_graph;

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

template <typename Output, typename... Inputs>
class function_graph {
public:
  using NodeVariant = std::variant<Source, InternalNode, Sink>;

  friend class constructing_graph<Inputs...>;

  const std::vector<NodeVariant>& nodes() const { return _nodes; }
  const std::vector<std::string>& names() const { return _names; }
  const small_vec_base<std::pair<int, int>>& outputs(int i) const {
    return _outputs[i];
  }

private:
  function_graph(std::vector<NodeVariant> nodes, std::vector<std::string> names,
                 std::vector<small_vec<std::pair<int, int>, 3>> outputs)
      : _nodes(std::move(nodes)), _names(std::move(names)),
        _outputs(std::move(outputs)) {}

  std::vector<NodeVariant> _nodes;
  std::vector<std::string> _names;
  // node -> id, idx
  std::vector<small_vec<std::pair<int, int>, 3>> _outputs;
};

template <typename... Inputs>
class constructing_graph {
  using NodeVariant = std::variant<Source, InternalNode, Sink>;

public:
  template <typename F>
  constructing_graph& add(F f, std::string name) {
    return add_internal(make_any_function(std::move(f)), std::move(name), {});
  }

  template <typename F>
  constructing_graph& add(F f, std::string name,
                          small_vec<std::string, 3> inputs) {

    return add_internal(
        make_any_function(std::move(f)), std::move(name),
        util::map<small_vec<int, 3>>(
            std::move(inputs),
            [this](const std::string_view& name) { return idx_of(name); }));
  }

  template <typename Output, typename... InnerInputs>
  constructing_graph& add(function_graph<Output, InnerInputs...> g,
                          std::string name, small_vec<std::string, 3> inputs) {

    const int index_offset = _nodes.size() - sizeof...(InnerInputs);

    boost::for_each(
        boost::combine(g.nodes(), g.names()),
        [this, name, inputs, index_offset](const auto& tuple) {
          const auto& variant = boost::get<0>(tuple);
          const auto& inner_name = boost::get<1>(tuple);

          if(std::holds_alternative<InternalNode>(variant)) {
            const auto& node = std::get<InternalNode>(variant);
            add_internal(
                node.func, name + "." + inner_name,
                util::map<small_vec<int, 3>>(node.inputs, [&](int old_input) {
                  if(old_input < static_cast<int>(sizeof...(InnerInputs))) {
                    return idx_of(inputs[old_input]);
                  } else {
                    return old_input + index_offset;
                  }
                }));
          } else if(std::holds_alternative<Sink>(variant)) {
            const auto& sink = std::get<Sink>(variant);
            _aliases.emplace(name, sink.node_idx + index_offset);
          }
        });

    return *this;
  }

  template <typename Output>
  function_graph<Output, Inputs...> output(const std::string_view& name) {
    return output_internal<Output>(idx_of(name));
  }

private:
  std::vector<NodeVariant> _nodes;
  std::vector<std::string> _names;
  std::vector<small_vec<std::pair<int, int>, 3>> _outputs;
  std::unordered_map<std::string, int> _aliases;

  constructing_graph(std::array<std::string, sizeof...(Inputs)> names)
      : _nodes(
            util::make_std_vector<NodeVariant>(Source(make_type<Inputs>())...)),
        _names(), _outputs(sizeof...(Inputs)) {
    _names.reserve(sizeof...(Inputs));
    for(auto&& name : std::move(names)) {
      assert(!name_exists(name));
      _names.push_back(name);
    }
  }

  constructing_graph(int num_inputs)
      : _nodes(
            util::make_std_vector<NodeVariant>(Source(make_type<Inputs>())...)),
        _names(), _outputs(sizeof...(Inputs)) {
    _names.reserve(sizeof...(Inputs));
    for(int i = 0; i < num_inputs; i++) {
      _names.push_back(std::to_string(i));
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
    return std::find(_names.begin(), _names.end(), new_name) != _names.end() ||
           _aliases.find(std::string(new_name)) != _aliases.end();
  }

  int idx_of(const std::string_view& name) const {
    auto it = std::find(_names.begin(), _names.end(), name);
    if(it != _names.end()) {
      return std::distance(_names.begin(), it);
    }

    auto alias_it = _aliases.find(std::string(name));
    assert(alias_it != _aliases.end());

    return alias_it->second;
  }

  constructing_graph& add_internal(any_function f, std::string name,
                                   small_vec<int, 3> inputs) {
    assert(!name_exists(name));

    auto node = InternalNode(std::move(f), std::move(inputs));

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
  function_graph<Output, Inputs...> output_internal(int idx) {
    _outputs[idx].emplace_back(_nodes.size(), 0);
    _nodes.emplace_back(Sink(idx));
    _names.emplace_back(".out");
    _outputs.emplace_back();

    return function_graph<Output, Inputs...>(
        std::move(_nodes), std::move(_names), std::move(_outputs));
  }

  template <typename... Ts>
  friend constructing_graph<Ts...>
  make_graph(std::array<std::string, sizeof...(Ts)> names);

  friend constructing_graph<> make_graph();

  template <typename Out, typename... Is, typename... Fs>
  friend function_graph<Out, Is...> make_pipeline(Fs... fs);
};

template <typename... Inputs>
inline constructing_graph<Inputs...>
make_graph(std::array<std::string, sizeof...(Inputs)> names) {
  return constructing_graph<Inputs...>(std::move(names));
}

inline constructing_graph<> make_graph() { return constructing_graph<>({}); }

template <typename Output, typename... Inputs, typename... Fs>
inline function_graph<Output, Inputs...> make_pipeline(Fs... fs) {
  auto g = constructing_graph<Inputs...>(sizeof...(Inputs));

  auto funcs =
      util::make_std_vector<any_function>(make_any_function(std::move(fs))...);

  small_vec<int, 3> first_inputs(sizeof...(Inputs));
  std::iota(first_inputs.begin(), first_inputs.end(), 0);
  g.add_internal(std::move(funcs[0]), std::to_string(sizeof...(Inputs)),
                 std::move(first_inputs));

  for(int i = 1; i < static_cast<int>(sizeof...(Fs)); i++) {
    g.add_internal(std::move(funcs[i]), std::to_string(sizeof...(Inputs) + i),
                   {static_cast<int>(sizeof...(Inputs)) + i - 1});
  }

  return g.template output_internal<Output>(sizeof...(Inputs) + sizeof...(Fs) -
                                            1);
}

} // namespace anyf

#endif