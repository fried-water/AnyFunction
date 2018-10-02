#include "any_function.h"

#include "test.h"

using namespace anyf;

void test_input_val() {
  auto test_val = [](Sentinal a, int copies, int moves) {
    assert(a.moves == moves);
    assert(a.copies == copies);
  };

  auto func = make_any_function(test_val);

  Sentinal x;
  const Sentinal c_x;

  test_val(x, 1, 0);
  test_val(std::move(x), 0, 1);
  test_val(c_x, 1, 0);
  test_val(std::move(c_x), 1, 0);

  func.invoke<void>(x, 1, 0);
  func.invoke<void>(std::move(x), 0, 1);
  func.invoke<void>(c_x, 1, 0);
  func.invoke<void>(std::move(c_x), 1, 0);
}

void test_input_ref() {
  auto test_ref = [](Sentinal& a, int copies, int moves) {
    assert(a.moves == moves);
    assert(a.copies == copies);
  };

  auto func = make_any_function(test_ref);

  Sentinal x;
  const Sentinal c_x;

  test_ref(x, 0, 0);
  // test_ref(std::move(x), 0, 0);   // Not allowed
  // test_ref(c_x, 0, 0);            // Not allowed
  // test_ref(std::move(c_x), 0, 0); // Not allowed

  func.invoke<void>(x, 0, 0);
  assert_throws<bad_any_invocation>(
      [&]() { func.invoke<void>(std::move(x), 0, 0); });
  // assert_throws<bad_any_invocation>([&](){func.invoke<void>(c_x, 0, 0);}); //
  // TODO throw exception
  // assert_throws<bad_any_invocation>([&](){func.invoke<void>(std::move(c_x),
  // 0, 0);}); // TODO throw exception
}

void test_input_const_ref() {
  auto test_const_ref = [](const Sentinal& a, int copies, int moves) {
    assert(a.moves == moves);
    assert(a.copies == copies);
  };

  auto func = make_any_function(test_const_ref);

  Sentinal x;
  const Sentinal c_x;

  test_const_ref(x, 0, 0);
  test_const_ref(std::move(x), 0, 0);
  test_const_ref(c_x, 0, 0);
  test_const_ref(std::move(c_x), 0, 0);

  func.invoke<void>(x, 0, 0);
  func.invoke<void>(std::move(x), 0, 0);
  func.invoke<void>(c_x, 0, 0);
  func.invoke<void>(std::move(c_x), 0, 0);
}

void test_input_rref() {
  auto test_rref = [](Sentinal&& a, int copies, int moves) {
    assert(a.moves == moves);
    assert(a.copies == copies);
  };

  auto func = make_any_function(test_rref);

  Sentinal x;
  const Sentinal c_x;

  // test_rref(x, 0, 0); // Not allowed
  test_rref(std::move(x), 0, 0);
  // test_rref(c_x, 0, 0); // Not allowed
  // test_rref(std::move(c_x), 0, 0); // Not allowed

  assert_throws<bad_any_invocation>([&]() { func.invoke<void>(x, 0, 0); });
  func.invoke<void>(std::move(x), 0, 0);
  assert_throws<bad_any_invocation>([&]() { func.invoke<void>(c_x, 0, 0); });
  assert_throws<bad_any_invocation>(
      [&]() { func.invoke<void>(std::move(c_x), 0, 0); });
}

void test_input_const_rref() {
  auto test_const_rref = [](const Sentinal&& a, int copies, int moves) {
    assert(a.moves == moves);
    assert(a.copies == copies);
  };

  auto func = make_any_function(test_const_rref);

  Sentinal x;
  const Sentinal c_x;

  // test_const_rref(x, 0, 0); // Not allowed
  test_const_rref(std::move(x), 0, 0);
  // test_const_rref(c_x, 0, 0); // Not allowed
  test_const_rref(std::move(c_x), 0, 0);

  assert_throws<bad_any_invocation>([&]() { func.invoke<void>(x, 0, 0); });
  func.invoke<void>(std::move(x), 0, 0);
  assert_throws<bad_any_invocation>([&]() { func.invoke<void>(c_x, 0, 0); });
  // func.invoke<void>(std::move(c_x), 0, 0); // TODO need to fix this
}

void any_function_test() {
  test_input_val();
  test_input_ref();
  test_input_const_ref();
  test_input_rref();
  test_input_const_rref();
}
