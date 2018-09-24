#ifndef EXECUTOR_H
#define EXECUTOR_H

#include "graph.h"

namespace comp {


template <typename Output, typename... Inputs>
Output execute_graph(const DependencyGraph& g, Inputs&&... inputs) {
  //TODO
}

}


#endif