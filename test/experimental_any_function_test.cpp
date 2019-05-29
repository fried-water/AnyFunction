#include "experimental/any_function.h"

#include "sentinal.h"

#define BOOST_TEST_DYN_LINK
#include <boost/test/unit_test.hpp>

using namespace anyf;

bool valid_exception(bad_any_invocation const&) { return true; }

BOOST_AUTO_TEST_CASE(test_input_val) {
  auto test_val = [](sentinal a, int copies, int moves) {
    BOOST_CHECK_EQUAL(moves, a.moves);
    BOOST_CHECK_EQUAL(copies, a.copies);
  };

  auto func = make_any_function(test_val);

  sentinal x;
  const sentinal c_x;

  test_val(x, 1, 0);
  test_val(std::move(x), 0, 1);
  test_val(c_x, 1, 0);
  test_val(std::move(c_x), 1, 0);

  func.invoke<void>(x, 1, 0);
  func.invoke<void>(std::move(x), 0, 1);
  func.invoke<void>(c_x, 1, 0);
  func.invoke<void>(std::move(c_x), 1, 0);
}

BOOST_AUTO_TEST_CASE(test_input_ref) {
  auto test_ref = [](sentinal& a, int copies, int moves) {
    BOOST_CHECK_EQUAL(moves, a.moves);
    BOOST_CHECK_EQUAL(copies, a.copies);
  };

  auto func = make_any_function(test_ref);

  sentinal x;
  const sentinal c_x;

  test_ref(x, 0, 0);
  // test_ref(std::move(x), 0, 0);   // Not allowed
  // test_ref(c_x, 0, 0);            // Not allowed
  // test_ref(std::move(c_x), 0, 0); // Not allowed

  func.invoke<void>(x, 0, 0);
  BOOST_CHECK_EXCEPTION(func.invoke<void>(std::move(x), 0, 0),
                        bad_any_invocation, valid_exception);
  // BOOST_CHECK_EXCEPTION(func.invoke<void>(c_x, 0, 0), bad_any_invocation,
  // valid_exception); BOOST_CHECK_EXCEPTION(func.invoke<void>(std::move(c_x),
  // 0, 0), bad_any_invocation, valid_exception);
  // TODO throw exception
}

BOOST_AUTO_TEST_CASE(test_input_const_ref) {
  auto test_const_ref = [](const sentinal& a, int copies, int moves) {
    BOOST_CHECK_EQUAL(moves, a.moves);
    BOOST_CHECK_EQUAL(copies, a.copies);
  };

  auto func = make_any_function(test_const_ref);

  sentinal x;
  const sentinal c_x;

  test_const_ref(x, 0, 0);
  test_const_ref(std::move(x), 0, 0);
  test_const_ref(c_x, 0, 0);
  test_const_ref(std::move(c_x), 0, 0);

  func.invoke<void>(x, 0, 0);
  func.invoke<void>(std::move(x), 0, 0);
  func.invoke<void>(c_x, 0, 0);
  func.invoke<void>(std::move(c_x), 0, 0);
}

BOOST_AUTO_TEST_CASE(test_input_rref) {
  auto test_rref = [](sentinal&& a, int copies, int moves) {
    BOOST_CHECK_EQUAL(moves, a.moves);
    BOOST_CHECK_EQUAL(copies, a.copies);
  };

  auto func = make_any_function(test_rref);

  sentinal x;
  const sentinal c_x;

  // test_rref(x, 0, 0); // Not allowed
  test_rref(std::move(x), 0, 0);
  // test_rref(c_x, 0, 0); // Not allowed
  // test_rref(std::move(c_x), 0, 0); // Not allowed

  BOOST_CHECK_EXCEPTION(func.invoke<void>(x, 0, 0), bad_any_invocation,
                        valid_exception);
  func.invoke<void>(std::move(x), 0, 0);
  BOOST_CHECK_EXCEPTION(func.invoke<void>(c_x, 0, 0), bad_any_invocation,
                        valid_exception);
  BOOST_CHECK_EXCEPTION(func.invoke<void>(std::move(c_x), 0, 0),
                        bad_any_invocation, valid_exception);
}

BOOST_AUTO_TEST_CASE(test_input_const_rref) {
  auto test_const_rref = [](const sentinal&& a, int copies, int moves) {
    BOOST_CHECK_EQUAL(moves, a.moves);
    BOOST_CHECK_EQUAL(copies, a.copies);
  };

  auto func = make_any_function(test_const_rref);

  sentinal x;
  const sentinal c_x;

  // test_const_rref(x, 0, 0); // Not allowed
  test_const_rref(std::move(x), 0, 0);
  // test_const_rref(c_x, 0, 0); // Not allowed
  test_const_rref(std::move(c_x), 0, 0);

  BOOST_CHECK_EXCEPTION(func.invoke<void>(x, 0, 0), bad_any_invocation,
                        valid_exception);
  func.invoke<void>(std::move(x), 0, 0);
  BOOST_CHECK_EXCEPTION(func.invoke<void>(c_x, 0, 0), bad_any_invocation,
                        valid_exception);
  // BOOST_CHECK_EXCEPTION(func.invoke<void>(std::move(c_x), 0, 0),
  // bad_any_invocation, valid_exception);
  // TODO
}
