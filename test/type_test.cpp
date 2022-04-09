#include "type.h"

#include <boost/test/unit_test.hpp>

using namespace anyf;

namespace my_namespace {
struct MyType {};
} // namespace my_namespace

namespace {

struct MyType {};

struct ImplicitCopyOnlyType {
  ImplicitCopyOnlyType(ImplicitCopyOnlyType const&){};
  // Implicitly deleted
  // ImplicitCopyOnlyType(ImplicitCopyOnlyType&&) = delete;
};

struct ExplicitCopyOnlyType {
  ExplicitCopyOnlyType(ExplicitCopyOnlyType const&){};
  ExplicitCopyOnlyType(ExplicitCopyOnlyType&&) = delete;
};

struct MoveOnlyType {
  MoveOnlyType(MoveOnlyType const&) = delete;
  MoveOnlyType(MoveOnlyType&&) = default;
};

struct NoCopyNoMoveType {
  NoCopyNoMoveType(NoCopyNoMoveType const&) = delete;
  NoCopyNoMoveType(NoCopyNoMoveType&&) = delete;
};

} // namespace

BOOST_AUTO_TEST_CASE(test_const_types) {
  BOOST_CHECK_EQUAL(false, make_type<int>().is_const());
  BOOST_CHECK_EQUAL(false, make_type<std::string>().is_const());
  BOOST_CHECK_EQUAL(false, make_type<MyType>().is_const());

  BOOST_CHECK_EQUAL(true, make_type<const int>().is_const());
  BOOST_CHECK_EQUAL(true, make_type<const std::string>().is_const());
  BOOST_CHECK_EQUAL(true, make_type<const MyType>().is_const());

  BOOST_CHECK_EQUAL(false, make_type<int&>().is_const());
  BOOST_CHECK_EQUAL(false, make_type<std::string&>().is_const());
  BOOST_CHECK_EQUAL(false, make_type<MyType&>().is_const());

  BOOST_CHECK_EQUAL(true, make_type<const int&>().is_const());
  BOOST_CHECK_EQUAL(true, make_type<const std::string&>().is_const());
  BOOST_CHECK_EQUAL(true, make_type<const MyType&>().is_const());
}

BOOST_AUTO_TEST_CASE(test_ref_types) {
  BOOST_CHECK_EQUAL(false, make_type<int>().is_ref());
  BOOST_CHECK_EQUAL(false, make_type<std::string>().is_ref());
  BOOST_CHECK_EQUAL(false, make_type<MyType>().is_ref());

  BOOST_CHECK_EQUAL(false, make_type<const int>().is_ref());
  BOOST_CHECK_EQUAL(false, make_type<const std::string>().is_ref());
  BOOST_CHECK_EQUAL(false, make_type<const MyType>().is_ref());

  BOOST_CHECK_EQUAL(true, make_type<int&>().is_ref());
  BOOST_CHECK_EQUAL(true, make_type<std::string&>().is_ref());
  BOOST_CHECK_EQUAL(true, make_type<MyType&>().is_ref());

  BOOST_CHECK_EQUAL(true, make_type<const int&>().is_ref());
  BOOST_CHECK_EQUAL(true, make_type<const std::string&>().is_ref());
  BOOST_CHECK_EQUAL(true, make_type<const MyType&>().is_ref());
}

BOOST_AUTO_TEST_CASE(test_is_copy_constructible) {
  BOOST_CHECK_EQUAL(true, make_type<MyType>().is_copy_constructible());
  BOOST_CHECK_EQUAL(true, make_type<ImplicitCopyOnlyType>().is_copy_constructible());
  BOOST_CHECK_EQUAL(true, make_type<ExplicitCopyOnlyType>().is_copy_constructible());
  BOOST_CHECK_EQUAL(false, make_type<MoveOnlyType>().is_copy_constructible());
  BOOST_CHECK_EQUAL(false, make_type<NoCopyNoMoveType>().is_copy_constructible());
}

BOOST_AUTO_TEST_CASE(test_is_move_constructible) {
  BOOST_CHECK_EQUAL(true, make_type<MyType>().is_move_constructible());
  BOOST_CHECK_EQUAL(true, make_type<ImplicitCopyOnlyType>().is_move_constructible());
  // why does this make a difference?
  BOOST_CHECK_EQUAL(false, make_type<ExplicitCopyOnlyType>().is_move_constructible());
  BOOST_CHECK_EQUAL(true, make_type<MoveOnlyType>().is_move_constructible());
  BOOST_CHECK_EQUAL(false, make_type<NoCopyNoMoveType>().is_move_constructible());
}

BOOST_AUTO_TEST_CASE(test_name) {
  BOOST_CHECK_EQUAL("int", make_type<int>().name());
  BOOST_CHECK_EQUAL("const int", make_type<const int>().name());
  BOOST_CHECK_EQUAL("int&", make_type<int&>().name());
  BOOST_CHECK_EQUAL("const int&", make_type<int const&>().name());

  BOOST_CHECK_EQUAL("my_namespace::MyType", make_type<my_namespace::MyType>().name());
  BOOST_CHECK_EQUAL("const my_namespace::MyType", make_type<const my_namespace::MyType>().name());
  BOOST_CHECK_EQUAL("my_namespace::MyType&", make_type<my_namespace::MyType&>().name());
  BOOST_CHECK_EQUAL("const my_namespace::MyType&", make_type<my_namespace::MyType const&>().name());
}
