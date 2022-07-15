#include "anyf/type.h"

#include <boost/test/unit_test.hpp>

using namespace anyf;

namespace my_namespace {
struct MyType {};
} // namespace my_namespace

namespace {

struct MyType {};

struct ImplicitCopyOnlyType {
  ImplicitCopyOnlyType(ImplicitCopyOnlyType const&) {}
  // Implicitly deleted
  // ImplicitCopyOnlyType(ImplicitCopyOnlyType&&) = delete;
};

struct MoveOnlyType {
  MoveOnlyType(MoveOnlyType const&) = delete;
  MoveOnlyType(MoveOnlyType&&) = default;
};

} // namespace

BOOST_AUTO_TEST_CASE(test_type_equality) {
  BOOST_CHECK(make_type_properties(Type<int>{}) == make_type_properties(Type<int>()));
  BOOST_CHECK(make_type_properties(Type<int&&>()) == make_type_properties(Type<int>()));
  BOOST_CHECK(make_type_properties(Type<const int&>()) == make_type_properties(Type<const int&>()));
  BOOST_CHECK(make_type_properties(Type<int>()) != make_type_properties(Type<float>()));

  BOOST_CHECK(make_type_properties(Type<int>()).id == make_type_properties(Type<const int&>()).id);

  BOOST_CHECK(make_type_properties(Type<MyType>()) == make_type_properties(Type<MyType>()));
  BOOST_CHECK(make_type_properties(Type<MyType>()) != make_type_properties(Type<my_namespace::MyType>()));
}

BOOST_AUTO_TEST_CASE(test_is_value_types) {
  BOOST_CHECK_EQUAL(true, make_type_properties(Type<int>()).value);
  BOOST_CHECK_EQUAL(true, make_type_properties(Type<std::string>()).value);
  BOOST_CHECK_EQUAL(true, make_type_properties(Type<MyType>()).value);

  BOOST_CHECK_EQUAL(true, make_type_properties(Type<int&&>()).value);
  BOOST_CHECK_EQUAL(true, make_type_properties(Type<std::string&&>()).value);
  BOOST_CHECK_EQUAL(true, make_type_properties(Type<MyType&&>()).value);

  BOOST_CHECK_EQUAL(false, make_type_properties(Type<const int&>()).value);
  BOOST_CHECK_EQUAL(false, make_type_properties(Type<const std::string&>()).value);
  BOOST_CHECK_EQUAL(false, make_type_properties(Type<const MyType&>()).value);
}

BOOST_AUTO_TEST_CASE(test_is_copy_constructible) {
  BOOST_CHECK_EQUAL(true, is_copyable(type_id(Type<MyType>())));
  BOOST_CHECK_EQUAL(true, is_copyable(type_id(Type<ImplicitCopyOnlyType>())));
  BOOST_CHECK_EQUAL(false, is_copyable(type_id(Type<MoveOnlyType>())));
}
