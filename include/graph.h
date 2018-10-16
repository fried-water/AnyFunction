#ifndef GRAPH_H
#define GRAPH_H

#include "any_function.h"
#include "util.h"

#include <cctype>

#include <iostream>
#include <numeric>
#include <unordered_set>
#include <variant>

#include <boost/container/small_vector.hpp>
#include <boost/range/algorithm.hpp>
#include <boost/range/combine.hpp>

namespace anyf {

template <typename... Inputs>
class constructing_graph;

template <typename Output, typename... Inputs>
class function_graph;

enum class pass_by { move, copy, ref, def };

namespace graph {
struct edge {
  int dst_id;
  int arg_idx;
  pass_by pb;

  edge(int dst_id, int arg_idx, pass_by pb)
      : dst_id(dst_id), arg_idx(arg_idx), pb(pb) {}
};

struct source {
  std::type_index type;
  source(std::type_index type) : type(type) {}
};

struct sink {
  std::type_index type;
  int input_id;
  sink(std::type_index type, int input_id) : type(type), input_id(input_id) {}
};

struct node {
  any_function func;
  small_vec3<int> inputs;

  node(any_function func, small_vec3<int> inputs)
      : func(std::move(func)), inputs(std::move(inputs)) {}
};

inline bool is_valid_name(const std::string_view& name) {
  return name.size() != 0 && std::isalpha(name.front()) &&
         std::all_of(name.begin(), name.end(),
                     [](char c) { return std::isalnum(c) || c == '_'; });
}
} // namespace graph

template <typename Output, typename... Inputs>
class function_graph {
public:
  using NodeVariant = std::variant<graph::source, graph::node, graph::sink>;

  friend class constructing_graph<Inputs...>;

  const std::vector<NodeVariant>& nodes() const { return _nodes; }
  const std::vector<std::string>& names() const { return _names; }
  const small_vec_base<graph::edge>& outputs(int i) const {
    return _outputs[i];
  }

  function_graph<Output, Inputs...>
  decorate(std::function<any_function::func_type(std::string name,
                                                 any_function::func_type func)>
               decorator) {
    std::vector<NodeVariant> nodes;
    nodes.reserve(_nodes.size());

    for(int i = 0; i < static_cast<int>(_nodes.size()); i++) {
      const auto& variant = _nodes[i];

      if(std::holds_alternative<graph::node>(variant)) {
        const auto& node = std::get<graph::node>(variant);

        nodes.emplace_back(
            graph::node(node.func.decorate([name = _names[i], decorator](
                                               any_function::func_type func) {
              return decorator(name, std::move(func));
            }),
                        node.inputs));
      } else {
        nodes.push_back(variant);
      }
    };

    return function_graph(std::move(nodes), _names, _outputs);
  }

private:
  function_graph(std::vector<NodeVariant> nodes, std::vector<std::string> names,
                 std::vector<small_vec3<graph::edge>> outputs)
      : _nodes(std::move(nodes)), _names(std::move(names)),
        _outputs(std::move(outputs)) {}

  std::vector<NodeVariant> _nodes;
  std::vector<std::string> _names;
  // node -> id, idx
  std::vector<small_vec3<graph::edge>> _outputs;
};

template <typename... Inputs>
class constructing_graph {
  using NodeVariant = std::variant<graph::source, graph::node, graph::sink>;

public:
  template <typename F>
  constructing_graph& add(F f, std::string name,
                          small_vec3<std::string> inputs = {},
                          small_vec3<pass_by> pass_bys = {}) {

    if(!graph::is_valid_name(name)) {
      std::cout << "Invalid name: " << name << "\n";
      assert(false);
    }

    return add_internal(
        make_any_function(std::move(f)), std::move(name),
        util::map<small_vec3<int>>(
            inputs,
            [this](const std::string_view& name) { return idx_of(name); }),
        std::move(pass_bys));
  }

  template <typename Output, typename... InnerInputs>
  constructing_graph& add(const function_graph<Output, InnerInputs...>& graph,
                          std::string name, small_vec3<std::string> inputs) {

    const int index_offset = _nodes.size() - sizeof...(InnerInputs);

    for(int i = 0; i < static_cast<int>(graph.nodes().size()); i++) {
      const auto& variant = graph.nodes()[i];
      const auto& inner_name = graph.names()[i];

      if(std::holds_alternative<graph::node>(variant)) {
        const auto& node = std::get<graph::node>(variant);

        small_vec3<pass_by> pass_bys(node.inputs.size(), pass_by::def);
        for(int j = 0; j < i; j++) {
          for(const graph::edge& edge : graph.outputs(j)) {
            if(edge.dst_id == i) {
              pass_bys[edge.arg_idx] =
                  std::holds_alternative<graph::source>(graph.nodes()[j])
                      ? pass_by::def // TODO use passed in value
                      : edge.pb;
            }
          }
        }

        add_internal(node.func, name + "." + inner_name,
                     util::map<small_vec3<int>>(
                         node.inputs,
                         [&](int old_input) {
                           if(old_input <
                              static_cast<int>(sizeof...(InnerInputs))) {
                             return idx_of(inputs[old_input]);
                           } else {
                             return old_input + index_offset;
                           }
                         }),
                     std::move(pass_bys));
      }
    }

    _aliases.emplace(std::move(name),
                     std::get<graph::sink>(graph.nodes().back()).input_id +
                         index_offset);

    return *this;
  }

  template <typename Output>
  function_graph<Output, Inputs...> output(const std::string_view& name,
                                           pass_by pb = pass_by::def) {
    int idx = idx_of(name);

    if(pb == pass_by::move) {
      assert(_moved_outputs.find(idx) == _moved_outputs.end());
      _moved_outputs.insert(idx);
    }

    _outputs[idx].emplace_back(_nodes.size(), 0, pb);
    _nodes.emplace_back(graph::sink(get_type(idx), idx));

    return function_graph<Output, Inputs...>(
        std::move(_nodes), std::move(_names),
        fix_default_pass_bys(std::move(_outputs)));
  }

private:
  std::vector<NodeVariant> _nodes;
  std::vector<std::string> _names;
  std::vector<small_vec3<graph::edge>> _outputs;
  std::unordered_map<std::string, int> _aliases;
  std::unordered_set<int> _moved_outputs;

  constructing_graph(std::array<std::string, sizeof...(Inputs)> names)
      : _nodes(util::make_std_vector<NodeVariant>(
            graph::source(std::type_index(typeid(Inputs)))...)),
        _names(), _outputs(sizeof...(Inputs)) {
    _names.reserve(sizeof...(Inputs));
    for(auto&& name : std::move(names)) {
      if(!graph::is_valid_name(name)) {
        std::cout << "Invalid input name: " << name << "\n";
        assert(false);
      }
      assert(!name_exists("." + name));
      _names.push_back("." + name);
    }
  }

  std::type_index get_type(int i) const {
    return std::visit(
        [](const auto& arg) {
          using T = std::decay_t<decltype(arg)>;
          if constexpr(std::is_same_v<T, graph::node>) {
            return arg.func.output_type();
          } else {
            return arg.type;
          }
        },
        _nodes[i]);
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

  bool name_exists(const std::string_view& new_name) const {
    return std::find(_names.begin(), _names.end(), new_name) != _names.end() ||
           _aliases.find(std::string(new_name)) != _aliases.end();
  }

  std::vector<small_vec3<graph::edge>>
  fix_default_pass_bys(std::vector<small_vec3<graph::edge>> all_outputs) {
    boost::for_each(all_outputs, [this](auto& outputs) {
      // Set all defaults taken by ref as ref
      boost::for_each(outputs, [this](graph::edge& edge) {
        std::visit(
            [&edge](const auto& arg) {
              using T = std::decay_t<decltype(arg)>;
              if constexpr(std::is_same_v<T, graph::node>) {
                if(arg.func.input_by_cref()[edge.arg_idx]) {
                  assert(edge.pb == pass_by::def || edge.pb == pass_by::ref);
                  edge.pb = pass_by::ref;
                } else {
                  assert(edge.pb != pass_by::ref);
                }
              } else {
                assert(edge.pb != pass_by::ref);
              }
            },
            _nodes[edge.dst_id]);
      });

      // Check if the value is already moved or has a ref
      bool can_move = std::none_of(
          outputs.begin(), outputs.end(), [](const graph::edge& edge) {
            return edge.pb == pass_by::ref || edge.pb == pass_by::move;
          });

      // If not, set first default to be move
      if(can_move) {
        auto first_def_it =
            boost::find_if(outputs, [](const graph::edge& edge) {
              return edge.pb == pass_by::def;
            });

        if(first_def_it != outputs.end()) {
          first_def_it->pb = pass_by::move;
        }
      }

      // Make the rest of the defaults copies
      boost::for_each(outputs, [](graph::edge& edge) {
        if(edge.pb == pass_by::def) {
          edge.pb = pass_by::copy;
        }
      });
    });

    return all_outputs;
  }

  constructing_graph& add_internal(any_function f, std::string name,
                                   small_vec3<int> inputs,
                                   small_vec3<pass_by> pass_bys) {
    assert(!name_exists(name));
    assert(f.input_types().size() == inputs.size());
    assert(pass_bys.size() == 0 || pass_bys.size() == inputs.size());

    if(pass_bys.size() == 0) {
      pass_bys = small_vec3<pass_by>(inputs.size(), pass_by::def);
    }

    // Check if moved inputs have been moved already
    boost::for_each(
        boost::combine(pass_bys, inputs), [this](const auto& tuple) {
          auto pb = boost::get<0>(tuple);
          int input_idx = boost::get<1>(tuple);

          if(pb == pass_by::move) {
            assert(_moved_outputs.find(input_idx) == _moved_outputs.end());
            _moved_outputs.insert(input_idx);
          }
        });

    auto node = graph::node(std::move(f), std::move(inputs));

    for(int i = 0; i < static_cast<int>(node.inputs.size()); i++) {
      auto output_type = get_type(node.inputs[i]);
      auto input_type = node.func.input_types()[i];

      if(output_type != input_type) {
        std::cout << "Output : " << output_type.name() << " does not match "
                  << input_type.name() << std::endl;
        throw 0;
      }

      _outputs[node.inputs[i]].emplace_back(_nodes.size(), i, pass_bys[i]);
    }

    _nodes.push_back(std::move(node));
    _names.push_back(std::move(name));
    _outputs.emplace_back();
    return *this;
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
                [i = 0]() mutable { return "in" + std::to_string(i++); });

  auto g = constructing_graph<Inputs...>(input_names);
  g.add(std::get<0>(std::move(func_tuple)), names[0],
        util::map<small_vec3<std::string>>(
            input_names, [](const std::string& name) { return "." + name; }));

  util::tuple_for_each(util::drop_first(std::move(func_tuple)),
                       [&](int i, auto&& func) mutable {
                         g.add(func, names[i + 1], {names[i]});
                       });

  return std::move(g).template output<Output>(names.back());
}

} // namespace anyf

#endif