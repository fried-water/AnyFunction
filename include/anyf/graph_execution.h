#pragma once

#include "anyf/borrowed_future.h"
#include "anyf/future.h"
#include "anyf/graph.h"
#include "anyf/static_graph.h"
#include "anyf/util.h"

namespace anyf {

std::vector<Future> execute_graph(const FunctionGraph&, ExecutorRef, std::vector<Future>, std::vector<BorrowedFuture>);

std::vector<Any> execute_graph(const FunctionGraph&, ExecutorRef, std::vector<Any>);

template <typename... Outputs, typename... Inputs>
auto execute_graph(const StaticFunctionGraph<TypeList<Outputs...>, TypeList<std::decay_t<Inputs>...>>& g,
                   ExecutorRef executor,
                   Inputs&&... inputs) {
  return apply_range<sizeof...(Outputs)>(
    execute_graph(g, executor, make_vector<Any>(std::forward<Inputs>(inputs)...)),
    [](auto&&... anys) { return tuple_or_value(any_cast<Outputs>(std::move(anys))...); });
}

} // namespace anyf
