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
  BOOST_CHECK(Type(Ty<int>{}) == Type(Ty<int>()));
  BOOST_CHECK(Type(Ty<int&>()) == Type(Ty<int&>()));
  BOOST_CHECK(Type(Ty<const int>()) == Type(Ty<const int>()));
  BOOST_CHECK(Type(Ty<const int&>()) == Type(Ty<const int&>()));
  BOOST_CHECK(Type(Ty<int>()) != Type(Ty<float>()));
  BOOST_CHECK(Type(Ty<int>()) != Type(Ty<int&>()));

  BOOST_CHECK(Type(Ty<int>()).type_id() == Type(Ty<const int&>()).type_id());

  BOOST_CHECK(Type(Ty<MyType>()) == Type(Ty<MyType>()));
  BOOST_CHECK(Type(Ty<MyType>()) != Type(Ty<my_namespace::MyType>()));
}

BOOST_AUTO_TEST_CASE(test_const_types) {
  BOOST_CHECK_EQUAL(false, Type(Ty<int>()).is_const());
  BOOST_CHECK_EQUAL(false, Type(Ty<std::string>()).is_const());
  BOOST_CHECK_EQUAL(false, Type(Ty<MyType>()).is_const());

  BOOST_CHECK_EQUAL(true, Type(Ty<const int>()).is_const());
  BOOST_CHECK_EQUAL(true, Type(Ty<const std::string>()).is_const());
  BOOST_CHECK_EQUAL(true, Type(Ty<const MyType>()).is_const());

  BOOST_CHECK_EQUAL(false, Type(Ty<int&>()).is_const());
  BOOST_CHECK_EQUAL(false, Type(Ty<std::string&>()).is_const());
  BOOST_CHECK_EQUAL(false, Type(Ty<MyType&>()).is_const());

  BOOST_CHECK_EQUAL(true, Type(Ty<const int&>()).is_const());
  BOOST_CHECK_EQUAL(true, Type(Ty<const std::string&>()).is_const());
  BOOST_CHECK_EQUAL(true, Type(Ty<const MyType&>()).is_const());
}

BOOST_AUTO_TEST_CASE(test_ref_types) {
  BOOST_CHECK_EQUAL(false, Type(Ty<int>()).is_ref());
  BOOST_CHECK_EQUAL(false, Type(Ty<std::string>()).is_ref());
  BOOST_CHECK_EQUAL(false, Type(Ty<MyType>()).is_ref());

  BOOST_CHECK_EQUAL(false, Type(Ty<const int>()).is_ref());
  BOOST_CHECK_EQUAL(false, Type(Ty<const std::string>()).is_ref());
  BOOST_CHECK_EQUAL(false, Type(Ty<const MyType>()).is_ref());

  BOOST_CHECK_EQUAL(true, Type(Ty<int&>()).is_ref());
  BOOST_CHECK_EQUAL(true, Type(Ty<std::string&>()).is_ref());
  BOOST_CHECK_EQUAL(true, Type(Ty<MyType&>()).is_ref());

  BOOST_CHECK_EQUAL(true, Type(Ty<const int&>()).is_ref());
  BOOST_CHECK_EQUAL(true, Type(Ty<const std::string&>()).is_ref());
  BOOST_CHECK_EQUAL(true, Type(Ty<const MyType&>()).is_ref());
}

BOOST_AUTO_TEST_CASE(test_is_copy_constructible) {
  BOOST_CHECK_EQUAL(true, Type(Ty<MyType>()).is_copy_constructible());
  BOOST_CHECK_EQUAL(true, Type(Ty<ImplicitCopyOnlyType>()).is_copy_constructible());
  BOOST_CHECK_EQUAL(true, Type(Ty<ExplicitCopyOnlyType>()).is_copy_constructible());
  BOOST_CHECK_EQUAL(false, Type(Ty<MoveOnlyType>()).is_copy_constructible());
  BOOST_CHECK_EQUAL(false, Type(Ty<NoCopyNoMoveType>()).is_copy_constructible());
}

BOOST_AUTO_TEST_CASE(test_is_move_constructible) {
  BOOST_CHECK_EQUAL(true, Type(Ty<MyType>()).is_move_constructible());
  BOOST_CHECK_EQUAL(true, Type(Ty<ImplicitCopyOnlyType>()).is_move_constructible());
  // why does this make a difference?
  BOOST_CHECK_EQUAL(false, Type(Ty<ExplicitCopyOnlyType>()).is_move_constructible());
  BOOST_CHECK_EQUAL(true, Type(Ty<MoveOnlyType>()).is_move_constructible());
  BOOST_CHECK_EQUAL(false, Type(Ty<NoCopyNoMoveType>()).is_move_constructible());
}
