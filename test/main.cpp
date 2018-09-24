#include "any_function.h"

#include "graph.h"
#include "executor.h"

#include <iostream>

using namespace comp;

struct MyStruct {
  int64_t data = 0;
  int64_t data1[1500];


  MyStruct(int data) : data(data) {
    std::cout << "MyStruct(int)\n";
  }

  MyStruct(const MyStruct& x) : data(x.data) {
    std::cout << "MyStruct(const MyStruct&)\n";
  }

  MyStruct(MyStruct&& x) : data(x.data) {
    std::cout << "MyStruct(MyStruct&&)\n";
  }

  MyStruct& operator=(const MyStruct& x) {
    std::cout << "MyStruct = const MyStruct&\n";
    data = x.data;
    return *this;
  }

  MyStruct& operator=(MyStruct&& x) {
    std::cout << "MyStruct = MyStruct&&\n";
    data = x.data;
    return *this;
  }
};

MyStruct const_ref_func(const MyStruct& a) {
  std::cout << a.data << std::endl;
  return a;
}

int main() {
  auto func = [](MyStruct b) {
    std::cout << b.data << std::endl;
    return MyStruct(b.data* 3);
  };

  std::cout << "-----Component\n";
  auto bb = invoke<MyStruct>(make_any_function(func), MyStruct(5));
  std::cout << "-----Component 2\n";
  const auto& aa = invoke<MyStruct>(make_any_function(const_ref_func), MyStruct(5));
  std::cout << "-----Raw\n";
  auto cc = func(MyStruct(5));
  std::cout << "Hello World " << aa.data << " " << bb.data << " " << cc.data << "\n";

  ConstructingGraph g;

  auto in1 = g.input<MyStruct>();
  auto mid1 = g.add(func, in1);
  auto final_graph = g.output(mid1);

  return 0;
}