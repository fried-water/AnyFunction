#include "any_function.h"

#include "graph.h"
#include "test.h"
//#include "executor.h"
//#include "tasks.h"

#include <iostream>

using namespace anyf;

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
  any_function_test();

//   auto func = [](MyStruct b) {
//     std::cout << b.data << std::endl;
//     return MyStruct(b.data* 3);
//   };

//   std::cout << "-----Component\n";
//   auto bb = make_any_function(func).invoke<MyStruct>(MyStruct(5));
//   std::cout << "-----Component 2\n";
//   const auto& aa = make_any_function(const_ref_func).invoke<MyStruct>(MyStruct(5));
//   std::cout << "-----Raw\n";
//   auto cc = func(MyStruct(5));
//   std::cout << "Hello World " << aa.data << " " << bb.data << " " << cc.data << "\n";

//   ConstructingGraph g;

//   auto in1 = g.input<MyStruct>();
//   auto mid1 = g.add(func, in1);
//   auto final_graph = g.output(mid1);

//   // TaskSystem task_system;

//   // execute_graph<MyStruct>(final_graph, task_system, MyStruct(7));

//   int sum = 0;
//   for(int i = 0; i < 100000; i++) {
//     auto temp = MyStruct(3);
//     sum += temp.data;
//   }

  // return sum;
}