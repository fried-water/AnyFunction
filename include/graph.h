#ifndef GRAPH_H
#define GRAPH_H

#include "any_value_function.h"
#include "util.h"

#include <iostream>
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
  std::type_index type;
  Source(std::type_index type) : type(type) {}
};

struct InternalNode {
  any_value_function func;
  small_vec<int, 3> inputs;

  InternalNode(any_value_function func, small_vec<int, 3> inputs)
      : func(std::move(func)), inputs(std::move(inputs)) {}
};

template <typename Output, typename... Inputs>
class function_graph {
public:
  using NodeVariant = std::variant<Source, InternalNode>;

  friend class constructing_graph<Inputs...>;

  int graph_output() const { return _graph_output; }
  const std::vector<NodeVariant>& nodes() const { return _nodes; }
  const std::vector<std::string>& names() const { return _names; }
  const small_vec_base<std::pair<int, int>>& outputs(int i) const {
    return _outputs[i];
  }

  function_graph<Output, Inputs...>
  decorate(std::function<std::function<std::any(small_vec<std::any, 3>&&)>(
               std::string name, any_value_function func)>
               decorator) {
    std::vector<NodeVariant> nodes;
    nodes.reserve(_nodes.size());

    for(int i = 0; i < static_cast<int>(_nodes.size()); i++) {
      const auto& variant = _nodes[i];

      if(std::holds_alternative<InternalNode>(variant)) {
        const auto& node = std::get<InternalNode>(variant);
        const auto& input_types = node.func.input_types();
        small_vec<std::type_index, 3> new_input_types(input_types.begin(),
                                                      input_types.end());
        std::function<std::any(small_vec<std::any, 3> &&)> wrapped_function =
            decorator(_names[i], node.func);

        nodes.emplace_back(InternalNode(
            any_value_function(wrapped_function, node.func.output_type(),
                               std::move(new_input_types)),
            node.inputs));
      } else {
        nodes.push_back(variant);
      }
    }

    return function_graph(nodes, names(), _outputs, graph_output());
  }

private:
  function_graph(std::vector<NodeVariant> nodes, std::vector<std::string> names,
                 std::vector<small_vec<std::pair<int, int>, 3>> outputs,
                 int graph_output)
      : _nodes(std::move(nodes)), _names(std::move(names)),
        _outputs(std::move(outputs)), _graph_output(graph_output) {}

  std::vector<NodeVariant> _nodes;
  std::vector<std::string> _names;
  // node -> id, idx
  std::vector<small_vec<std::pair<int, int>, 3>> _outputs;
  int _graph_output;
};

template <typename... Inputs>
class constructing_graph {
  using NodeVariant = std::variant<Source, InternalNode>;

public:
  template <typename F>
  constructing_graph& add(F f, std::string name) {
    return add_internal(make_any_value_function(std::move(f)), std::move(name),
                        {});
  }

  template <typename F>
  constructing_graph& add(F f, std::string name,
                          small_vec<std::string, 3> inputs) {

    return add_internal(make_any_value_function(std::move(f)), std::move(name),
                        util::map<small_vec<int, 3>>(
                            inputs, [this](const std::string_view& name) {
                              return idx_of(name);
                            }));
  }

  template <typename Output, typename... InnerInputs>
  constructing_graph& add(const function_graph<Output, InnerInputs...>& graph,
                          std::string name, small_vec<std::string, 3> inputs) {

    const int index_offset = _nodes.size() - sizeof...(InnerInputs);

    boost::for_each(
        boost::combine(graph.nodes(), graph.names()), [&](const auto& tuple) {
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
          }
        });

    _aliases.emplace(std::move(name), graph.graph_output() + index_offset);

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
      : _nodes(util::make_std_vector<NodeVariant>(
            Source(std::type_index(typeid(Inputs)))...)),
        _names(), _outputs(sizeof...(Inputs)) {
    _names.reserve(sizeof...(Inputs));
    for(auto&& name : std::move(names)) {
      assert(!name_exists(name));
      _names.push_back(name);
    }
  }

  constructing_graph(int num_inputs)
      : _nodes(util::make_std_vector<NodeVariant>(
            Source(std::type_index(typeid(Inputs)))...)),
        _names(), _outputs(sizeof...(Inputs)) {
    _names.reserve(sizeof...(Inputs));
    for(int i = 0; i < num_inputs; i++) {
      _names.push_back(std::to_string(i));
    }
  }

  std::type_index get_type(int i) const {
    return std::visit(
        [this](const auto& arg) {
          using T = std::decay_t<decltype(arg)>;
          if constexpr(std::is_same_v<T, Source>) {
            return arg.type;
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

  constructing_graph& add_internal(any_value_function f, std::string name,
                                   small_vec<int, 3> inputs) {
    assert(!name_exists(name));

    auto node = InternalNode(std::move(f), std::move(inputs));

    for(int i = 0; i < static_cast<int>(node.inputs.size()); i++) {
      auto output_type = get_type(node.inputs[i]);
      auto input_type = node.func.input_types()[i];

      if(output_type != input_type) {
        std::cout << "Output : " << output_type.name() << " does not match "
                  << input_type.name() << std::endl;
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
    return function_graph<Output, Inputs...>(
        std::move(_nodes), std::move(_names), std::move(_outputs), idx);
  }

  template <typename... Ts>
  friend constructing_graph<Ts...>
  make_graph(std::array<std::string, sizeof...(Ts)> names);

  friend constructing_graph<> make_graph();

  template <typename Out, typename... Is, typename... Fs>
  friend function_graph<Out, Is...>
  make_pipeline(const std::vector<std::string>& names, std::tuple<Fs...> fs);
};

template <typename... Inputs>
constructing_graph<Inputs...>
make_graph(std::array<std::string, sizeof...(Inputs)> names) {
  return constructing_graph<Inputs...>(std::move(names));
}

inline constructing_graph<> make_graph() { return constructing_graph<>({}); }

template <typename Output, typename... Inputs, typename... Fs>
function_graph<Output, Inputs...>
make_pipeline(const std::vector<std::string>& names,
              std::tuple<Fs...> func_tuple) {
  static_assert(sizeof...(Fs) > 0);

  assert(names.size() == sizeof...(Fs));

  std::array<std::string, sizeof...(Inputs)> input_names;
  std::generate(input_names.begin(), input_names.end(),
                [i = 0]() mutable { return std::to_string(i++); });

  auto g = constructing_graph<Inputs...>(input_names);
  g.add(std::get<0>(std::move(func_tuple)), names[0],
        util::map<small_vec<std::string, 3>>(
            input_names, [](const std::string& name) { return name; }));

  util::tuple_for_each(util::drop_first(std::move(func_tuple)),
                       [&](int i, auto&& func) mutable {
                         g.add(func, names[i + 1], {names[i]});
                       });

  return g.template output<Output>(names.back());
}

} // namespace anyf

#endif