#ifndef DECORATED_GRAPH_H
#define DECORATED_GRAPH_H

#include "graph.h"

namespace anyf {

template <template <typename, typename, typename...> typename Wrapper, typename F, std::size_t... Is>
auto create_decorated_function(F f, std::string name, std::index_sequence<Is...>) {
  using tuple_args = typename traits::function_traits<F>::args;
  using ret_type = typename traits::function_traits<F>::return_type;

  return [f = std::move(f), name = std::move(name)](std::tuple_element_t<Is, tuple_args>... args) {
    return Wrapper<ret_type, F, std::tuple_element_t<Is, tuple_args>...>{}(f, name, std::move(args)...);
  };
}

template <template <typename, typename, typename...> typename Wrapper, typename... Inputs>
class decorated_graph {
public:
  template <typename F>
  decorated_graph& add(F f, std::string name) {
    auto&& wrapped_f = create_decorated_function<Wrapper>(std::move(f), name, std::make_index_sequence<traits::function_traits<F>::arity>());
    _graph.add(wrapped_f, std::move(name));
    return *this;
  }

  template <typename F>
  decorated_graph& add(F f, std::string name,
                          small_vec<std::string, 3> inputs) {
        auto&& wrapped_f = create_decorated_function<Wrapper>(std::move(f), name, std::make_index_sequence<traits::function_traits<F>::arity>());
    _graph.add(wrapped_f, std::move(name), std::move(inputs));

    return *this;
  }

  template <typename Output>
  function_graph<Output, Inputs...> output(const std::string_view& name) {
    return _graph.template output<Output>(name);
  }

private:
  constructing_graph<Inputs...> _graph;

  decorated_graph(std::array<std::string, sizeof...(Inputs)> names)
      : _graph(make_graph<Inputs...>(std::move(names))) { }

  template <template <typename, typename, typename...> typename W, typename... Ts>
  friend decorated_graph<W, Ts...>
  make_decorated_graph(std::array<std::string, sizeof...(Ts)> names);
};

template <template <typename, typename, typename...> typename Wrapper, typename... Inputs>
inline decorated_graph<Wrapper, Inputs...>
make_decorated_graph(std::array<std::string, sizeof...(Inputs)> names) {
  return decorated_graph<Wrapper, Inputs...>(std::move(names));
}

} // namespace anyf

#endif