#ifndef TEST_H
#define TEST_H

struct Sentinal {

  int copies = 0;
  int moves = 0;

  Sentinal() {}

  Sentinal(const Sentinal& x) : copies(x.copies), moves(x.moves) { ++copies; }

  Sentinal(Sentinal&& x) : copies(x.copies), moves(x.moves) { ++moves; }

  Sentinal& operator=(const Sentinal& x) = delete;
  Sentinal& operator=(Sentinal&& x) = delete;
};

template <typename Exception, typename F>
void assert_throws(F f) {
  bool exception_happened = false;

  try {
    f();
  } catch(Exception&) {
    exception_happened = true;
  }

  assert(exception_happened);
}

void any_function_test();
void stress_test();

#endif