#include "any_function.h"

#include "test.h"

using namespace anyf;

Sentinal my_func(Sentinal a, const Sentinal b, Sentinal& c, const Sentinal& d) {
  std::cout << a.data << " + " << b.data << " + " << c.data << " + " << d.data << "\n";
  return Sentinal(a.data + b.data + c.data + d.data);
}

Sentinal& my_func_ref(Sentinal& a) {
  std::cout << "ref: " << a.data << "\n";
  return a;
}

const Sentinal& my_func_const_ref(Sentinal& a) {
  std::cout << "ref: " << a.data << "\n";
  return a;
}


/*
Calling invoke()
Copy: 1
Copy: 0
0 + 1 + 2 + 3
Move: 6
Move: 6
ref: 6
ref: 6
Calling invoke_with_any()
Move: 1
Move: 0
Move: 1
Move: 0
0 + 1 + 2 + 3
Move: 6
Move: 6
ref: 0
ref: 1
*/

void any_function_test() {

  Sentinal w(0), x(1), y(2), z(3);
  
  any_function::vec_type any_vec{std::any(w), std::any(x), std::any(y), std::any(z)};
  any_function::vec_type any_vec2{std::any(w)};
  any_function::vec_type any_vec3{std::any(x)};

  std::cout << "Calling invoke()\n";

  auto any_func = make_any_function(my_func);
  auto any_func_ref = make_any_function(my_func_ref);
  auto any_func_cref = make_any_function(my_func_const_ref);

  Sentinal r =  any_func.invoke<Sentinal>(w, x, y, z);
  Sentinal& r2 = any_func_ref.invoke<Sentinal&>(r);
  const Sentinal& r3 = any_func_cref.invoke<const Sentinal&>(r2);

  std::cout << "Calling invoke_with_any()\n";

  Sentinal s_val = std::any_cast<Sentinal>(any_func.invoke_with_any(any_vec));
  Sentinal* s_ptr = std::any_cast<Sentinal*>(any_func_ref.invoke_with_any(any_vec2));
  const Sentinal* s_cptr = std::any_cast<const Sentinal*>(any_func_cref.invoke_with_any(any_vec3));
}