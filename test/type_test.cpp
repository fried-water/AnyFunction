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

BOOST_AUTO_TEST_CASE(test_type_equality) {
  BOOST_CHECK(TypeProperties(Type<int>{}) == TypeProperties(Type<int>()));
  BOOST_CHECK(TypeProperties(Type<int&>()) == TypeProperties(Type<int&>()));
  BOOST_CHECK(TypeProperties(Type<const int>()) == TypeProperties(Type<const int>()));
  BOOST_CHECK(TypeProperties(Type<const int&>()) == TypeProperties(Type<const int&>()));
  BOOST_CHECK(TypeProperties(Type<int>()) != TypeProperties(Type<float>()));
  BOOST_CHECK(TypeProperties(Type<int>()) != TypeProperties(Type<int&>()));

  BOOST_CHECK(TypeProperties(Type<int>()).type_id() == TypeProperties(Type<const int&>()).type_id());

  BOOST_CHECK(TypeProperties(Type<MyType>()) == TypeProperties(Type<MyType>()));
  BOOST_CHECK(TypeProperties(Type<MyType>()) != TypeProperties(Type<my_namespace::MyType>()));
}

BOOST_AUTO_TEST_CASE(test_const_types) {
  BOOST_CHECK_EQUAL(false, TypeProperties(Type<int>()).is_const());
  BOOST_CHECK_EQUAL(false, TypeProperties(Type<std::string>()).is_const());
  BOOST_CHECK_EQUAL(false, TypeProperties(Type<MyType>()).is_const());

  BOOST_CHECK_EQUAL(true, TypeProperties(Type<const int>()).is_const());
  BOOST_CHECK_EQUAL(true, TypeProperties(Type<const std::string>()).is_const());
  BOOST_CHECK_EQUAL(true, TypeProperties(Type<const MyType>()).is_const());

  BOOST_CHECK_EQUAL(false, TypeProperties(Type<int&>()).is_const());
  BOOST_CHECK_EQUAL(false, TypeProperties(Type<std::string&>()).is_const());
  BOOST_CHECK_EQUAL(false, TypeProperties(Type<MyType&>()).is_const());

  BOOST_CHECK_EQUAL(true, TypeProperties(Type<const int&>()).is_const());
  BOOST_CHECK_EQUAL(true, TypeProperties(Type<const std::string&>()).is_const());
  BOOST_CHECK_EQUAL(true, TypeProperties(Type<const MyType&>()).is_const());
}

BOOST_AUTO_TEST_CASE(test_ref_types) {
  BOOST_CHECK_EQUAL(false, TypeProperties(Type<int>()).is_ref());
  BOOST_CHECK_EQUAL(false, TypeProperties(Type<std::string>()).is_ref());
  BOOST_CHECK_EQUAL(false, TypeProperties(Type<MyType>()).is_ref());

  BOOST_CHECK_EQUAL(false, TypeProperties(Type<const int>()).is_ref());
  BOOST_CHECK_EQUAL(false, TypeProperties(Type<const std::string>()).is_ref());
  BOOST_CHECK_EQUAL(false, TypeProperties(Type<const MyType>()).is_ref());

  BOOST_CHECK_EQUAL(true, TypeProperties(Type<int&>()).is_ref());
  BOOST_CHECK_EQUAL(true, TypeProperties(Type<std::string&>()).is_ref());
  BOOST_CHECK_EQUAL(true, TypeProperties(Type<MyType&>()).is_ref());

  BOOST_CHECK_EQUAL(true, TypeProperties(Type<const int&>()).is_ref());
  BOOST_CHECK_EQUAL(true, TypeProperties(Type<const std::string&>()).is_ref());
  BOOST_CHECK_EQUAL(true, TypeProperties(Type<const MyType&>()).is_ref());
}

BOOST_AUTO_TEST_CASE(test_is_copy_constructible) {
  BOOST_CHECK_EQUAL(true, TypeProperties(Type<MyType>()).is_copy_constructible());
  BOOST_CHECK_EQUAL(true, TypeProperties(Type<ImplicitCopyOnlyType>()).is_copy_constructible());
  BOOST_CHECK_EQUAL(true, TypeProperties(Type<ExplicitCopyOnlyType>()).is_copy_constructible());
  BOOST_CHECK_EQUAL(false, TypeProperties(Type<MoveOnlyType>()).is_copy_constructible());
  BOOST_CHECK_EQUAL(false, TypeProperties(Type<NoCopyNoMoveType>()).is_copy_constructible());
}

BOOST_AUTO_TEST_CASE(test_is_move_constructible) {
  BOOST_CHECK_EQUAL(true, TypeProperties(Type<MyType>()).is_move_constructible());
  BOOST_CHECK_EQUAL(true, TypeProperties(Type<ImplicitCopyOnlyType>()).is_move_constructible());
  // why does this make a difference?
  BOOST_CHECK_EQUAL(false, TypeProperties(Type<ExplicitCopyOnlyType>()).is_move_constructible());
  BOOST_CHECK_EQUAL(true, TypeProperties(Type<MoveOnlyType>()).is_move_constructible());
  BOOST_CHECK_EQUAL(false, TypeProperties(Type<NoCopyNoMoveType>()).is_move_constructible());
}
