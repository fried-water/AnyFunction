#ifndef GRAPH_H
#define GRAPH_H

#include "any_function.h"
#include "util.h"

#include <cctype>

#include <iostream>
#include <optional>
#include <numeric>
#include <unordered_set>
#include <unordered_map>
#include <variant>

#include <boost/container/small_vector.hpp>
#include <boost/range/algorithm/transform.hpp>
#include <boost/range/algorithm/for_each.hpp>
#include <boost/range/algorithm/find_if.hpp>
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

struct func_node {
  any_function func;
  func_node(any_function func)
      : func(std::move(func)) {}
};

inline bool is_valid_name(const std::string_view& name) {
  return name.size() != 0 && std::isalpha(name.front()) &&
         std::all_of(name.begin(), name.end(),
                     [](char c) { return std::isalnum(c) || c == '_'; });
}
} // namespace graph

struct graph_node {
  std::variant<graph::source, graph::func_node, graph::sink> variant;
  std::string name;
  small_vec3<graph::edge> outputs;

  graph_node(graph::source src, std::string name)
      : variant(std::move(src)), name(std::move(name)) {}
  graph_node(graph::sink sink) : variant(std::move(sink)) {}
  graph_node(graph::func_node func_node, std::string name)
      : variant(std::move(func_node)), name(std::move(name)) {}

  std::type_index type() const {
    return std::visit(
        [](const auto& arg) {
          using T = std::decay_t<decltype(arg)>;
          if constexpr(std::is_same_v<T, graph::func_node>) {
            return arg.func.output_type();
          } else {
            return arg.type;
          }
        },
        variant);
  }
};

template <typename Output, typename... Inputs>
class function_graph {
public:
  friend class constructing_graph<Inputs...>;

  const std::vector<graph_node>& nodes() const { return _nodes; }

  // function_graph<Output, Inputs...>
  // decorate(std::function<any_function::func_type(std::string name,
  //                                                any_function::func_type
  //                                                func)>
  //              decorator) {
  //   std::vector<graph_node> nodes;
  //   nodes.reserve(_nodes.size());

  //   for(int i = 0; i < static_cast<int>(_nodes.size()); i++) {
  //     const auto& variant = _nodes[i];

  //     if(std::holds_alternative<graph::node>(variant)) {
  //       const auto& node = std::get<graph::node>(variant);

  //       nodes.emplace_back(
  //           graph::node(node.func.decorate([name = _names[i], decorator](
  //                                              any_function::func_type func)
  //                                              {
  //             return decorator(name, std::move(func));
  //           }),
  //                       node.inputs));
  //     } else {
  //       nodes.push_back(variant);
  //     }
  //   };

  //   return function_graph(std::move(nodes), _names, _outputs);
  // }

private:
  function_graph(std::vector<graph_node> nodes) : _nodes(std::move(nodes)) {}

  std::vector<graph_node> _nodes;
};

template <typename... Inputs>
class constructing_graph {
public:
  template <typename F>
  constructing_graph& add(F f, std::string name,
                          small_vec3<std::string> inputs = {},
                          small_vec3<pass_by> pass_bys = {}) {

    if(!graph::is_valid_name(name)) {
      std::cout << "Invalid name: " << name << "\n";
      throw 0;
    }

    return add_internal(
        make_any_function(std::move(f)), std::move(name),
        util::map<small_vec3<int>>(inputs,
                                   [this](const std::string_view& name) {
                                     auto opt_idx = idx_of(name);
                                     if(!opt_idx) {
                                       std::cout << "Cannot find: " << name
                                                 << std::endl;
                                       throw 0;
                                     }

                                     return *opt_idx;
                                   }),
        std::move(pass_bys));
  }

  template <typename Output, typename... InnerInputs>
  constructing_graph& add(const function_graph<Output, InnerInputs...>& graph,
                          std::string name, small_vec3<std::string> inputs) {

    const int index_offset = _nodes.size() - sizeof...(InnerInputs);

    for(int i = 0; i < static_cast<int>(graph.nodes().size()); i++) {
      const graph_node& node = graph.nodes()[i];

      if(std::holds_alternative<graph::func_node>(node.variant)) {
        const auto& func_node = std::get<graph::func_node>(node.variant);

        small_vec3<int> input_idxs;
        small_vec3<pass_by> pass_bys(func_node.func.input_types().size(), pass_by::def);
        for(int j = 0; j < i; j++) {
          for(const graph::edge& edge : graph.nodes()[j].outputs) {
            if(edge.dst_id == i) {
              input_idxs.push_back(j);
              pass_bys[edge.arg_idx] =
                  std::holds_alternative<graph::source>(
                      graph.nodes()[j].variant)
                      ? pass_by::def // TODO use passed in value
                      : edge.pb;
            }
          }
        }

        add_internal(func_node.func, name + "." + node.name,
                     util::map<small_vec3<int>>(
                         input_idxs,
                         [&](int old_input) {
                           if(old_input <
                              static_cast<int>(sizeof...(InnerInputs))) {
                             return *idx_of(inputs[old_input]);
                           } else {
                             return old_input + index_offset;
                           }
                         }),
                     std::move(pass_bys));
      }
    }

    _aliases.emplace(
        std::move(name),
        std::get<graph::sink>(graph.nodes().back().variant).input_id +
            index_offset);

    return *this;
  }

  template <typename Output>
  function_graph<Output, Inputs...> output(const std::string_view& name,
                                           pass_by pb = pass_by::def) {
    auto opt_idx = idx_of(name);
    if(!opt_idx) {
      std::cout << "Cannot find: " << name << std::endl;
      throw 0;
    }

    int idx = *opt_idx;

    if(pb == pass_by::move) {
      assert(_moved_outputs.find(idx) == _moved_outputs.end());
      _moved_outputs.insert(idx);
    }

    _nodes[idx].outputs.emplace_back(_nodes.size(), 0, pb);
    _nodes.emplace_back(graph::sink(_nodes[idx].type(), idx));

    return function_graph<Output, Inputs...>(
        fix_default_pass_bys(std::move(_nodes)));
  }

private:
  std::vector<graph_node> _nodes;
  std::unordered_map<std::string, int> _aliases;
  std::unordered_set<int> _moved_outputs;

  constructing_graph(std::array<std::string, sizeof...(Inputs)> names) {
    std::initializer_list<std::type_index> types{std::type_index(typeid(Inputs))...};

    _nodes.reserve(sizeof...(Inputs));
    boost::transform(
      boost::combine(names, types),
      std::back_inserter(_nodes),
      [this](const auto& tuple) {
        const auto& name = boost::get<0>(tuple);
        const auto& type = boost::get<1>(tuple);

      if(!graph::is_valid_name(name)) {
        std::cout << "Invalid input name: " << name << "\n";
        throw 0;
      }
      assert(!idx_of(name));
      return graph_node(graph::source(type), name);
    });
  }

  std::optional<int> idx_of(const std::string_view& name) const {
    auto it =
        std::find_if(_nodes.begin(), _nodes.end(),
                     [&name](const auto& node) { return node.name == name; });

    if(it != _nodes.end()) {
      return std::distance(_nodes.begin(), it);
    }

    auto alias_it = _aliases.find(std::string(name));
    if(alias_it != _aliases.end())
      return alias_it->second;
    else
      return std::nullopt;
  }

  std::vector<graph_node> fix_default_pass_bys(std::vector<graph_node> nodes) {
    boost::for_each(nodes, [&nodes](auto& node) {
      // Set all defaults taken by ref as ref
      boost::for_each(node.outputs, [&nodes](graph::edge& edge) {
        std::visit(
            [&edge](const auto& arg) {
              using T = std::decay_t<decltype(arg)>;
              if constexpr(std::is_same_v<T, graph::func_node>) {
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
            nodes[edge.dst_id].variant);
      });

      // Check if the value is already moved or has a ref
      bool can_move = std::none_of(node.outputs.begin(), node.outputs.end(),
                                   [](const graph::edge& edge) {
                                     return edge.pb == pass_by::ref ||
                                            edge.pb == pass_by::move;
                                   });

      // If not, set first default to be move
      if(can_move) {
        auto first_def_it =
            boost::find_if(node.outputs, [](const graph::edge& edge) {
              return edge.pb == pass_by::def;
            });

        if(first_def_it != node.outputs.end()) {
          first_def_it->pb = pass_by::move;
        }
      }

      // Make the rest of the defaults copies
      boost::for_each(node.outputs, [](graph::edge& edge) {
        if(edge.pb == pass_by::def) {
          edge.pb = pass_by::copy;
        }
      });
    });

    return nodes;
  }

  constructing_graph& add_internal(any_function f, std::string name,
                                   small_vec3<int> inputs,
                                   small_vec3<pass_by> pass_bys) {
    assert(!idx_of(name));
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

    int i = 0;
    boost::for_each(
      boost::combine(inputs, pass_bys, f.input_types()), [&i, this](const auto& tuple) {
      int input_id = boost::get<0>(tuple);
      pass_by pb = boost::get<1>(tuple);

      auto input_type = boost::get<2>(tuple);
      auto output_type = _nodes[input_id].type();
      
      if(output_type != input_type) {
        std::cout << "Output : " << output_type.name() << " does not match "
                  << input_type.name() << std::endl;
        throw 0;
      }

      _nodes[input_id].outputs.emplace_back(_nodes.size(), i++,
                                                  pb);
    });

    _nodes.emplace_back(graph::func_node(std::move(f)), std::move(name));
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
            input_names, [](const std::string& name) { return name; }));

  util::tuple_for_each(util::drop_first(std::move(func_tuple)),
                       [&](int i, auto&& func) mutable {
                         g.add(func, names[i + 1], {names[i]});
                       });

  return std::move(g).template output<Output>(names.back());
}

} // namespace anyf

#endif