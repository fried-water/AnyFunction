#include "any_function.h"

#include "test.h"

using namespace anyf;

Sentinal my_func(Sentinal a, const Sentinal b, Sentinal &c, const Sentinal &d) {
  std::cout << a.data << " + " << b.data << " + " << c.data << " + " << d.data
            << "\n";
  return Sentinal(a.data + b.data + c.data + d.data);
}

Sentinal &my_func_ref(Sentinal &a) {
  std::cout << "ref: " << a.data << "\n";
  return a;
}

const Sentinal &my_func_const_ref(Sentinal &a) {
  std::cout << "ref: " << a.data << "\n";
  return a;
}

void any_function_test() {

  Sentinal w(0), x(1), y(2), z(3);

  any_function::vec_type any_vec{std::any(w), std::any(x), std::any(y),
                                 std::any(z)};
  any_function::vec_type any_vec2{std::any(w)};
  any_function::vec_type any_vec3{std::any(x)};

  std::cout << "Calling invoke()\n";

  auto any_func = make_any_function(my_func);
  auto any_func_ref = make_any_function(my_func_ref);
  auto any_func_cref = make_any_function(my_func_const_ref);

  any_func.invoke<Sentinal>(w, x, y, z);
  any_func_ref.invoke<Sentinal &>(y);
  any_func_cref.invoke<const Sentinal &>(z);

  std::cout << "Calling invoke_with_any()\n";

  std::any_cast<Sentinal>(any_func.invoke_any(std::move(any_vec)));
  std::any_cast<Sentinal *>(any_func_ref.invoke_any(std::move(any_vec2)));
  std::any_cast<const Sentinal *>(
      any_func_cref.invoke_any(std::move(any_vec3)));
}