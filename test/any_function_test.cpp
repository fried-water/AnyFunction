#include "any_function.h"

#include "test.h"

using namespace anyf;

Sentinal my_func_value(Sentinal a, Sentinal b) {
  std::cout << a.data << " + " << b.data << "\n";
  return Sentinal(a.data + b.data);
}

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

void any_function_test() {

  Sentinal w(0), x(1), y(2), z(3);

  Sentinal r3 = make_any_function(my_func_value).invoke<Sentinal>(x, Sentinal(5));
  // Sentinal r =  make_any_function(my_func).invoke<Sentinal>(w, x, y, z);
  // Sentinal& r2 = make_any_function(my_func_ref).invoke<Sentinal&>(r);
  // const Sentinal& r3 = make_any_function(my_func_const_ref).invoke<const Sentinal&>(r2);


  (void)r3;
}