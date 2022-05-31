#include "any.h"

#include <boost/test/unit_test.hpp>

using namespace anyf;

namespace {

struct MoveOnlyType {
  MoveOnlyType() = default;
  MoveOnlyType(const MoveOnlyType&) = delete;
  MoveOnlyType(MoveOnlyType&&) = default;
  friend bool operator==(const MoveOnlyType&, const MoveOnlyType&) { return true; }
};

} // namespace

BOOST_AUTO_TEST_CASE(any_basic) {
  Any any;

  BOOST_CHECK(!any.has_value());

  any = 5;

  BOOST_CHECK(any.has_value());
  BOOST_CHECK(type_id(Type<int>{}) == any.type());
  BOOST_CHECK(5 == any_cast<int>(any));

  any = std::string("abc");

  BOOST_CHECK(any.has_value());
  BOOST_CHECK(type_id(Type<std::string>{}) == any.type());
  BOOST_CHECK("abc" == any_cast<std::string>(any));

  const Any any2 = any;

  BOOST_CHECK(any2.has_value());
  BOOST_CHECK(type_id(Type<std::string>{}) == any2.type());
  BOOST_CHECK("abc" == any_cast<std::string>(any2));

  any = {};

  BOOST_CHECK(!any.has_value());

  any = any2;

  BOOST_CHECK(any.has_value());
  BOOST_CHECK(type_id(Type<std::string>{}) == any.type());
  BOOST_CHECK("abc" == any_cast<std::string>(any));
}

BOOST_AUTO_TEST_CASE(any_move_only) {
  Any any = MoveOnlyType();

  BOOST_CHECK(any.has_value());
  BOOST_CHECK(type_id(Type<MoveOnlyType>{}) == any.type());
  BOOST_CHECK(MoveOnlyType() == any_cast<MoveOnlyType>(any));

  BOOST_CHECK_THROW(Any a = any, BadCopy);

  Any any2 = std::move(any);

  BOOST_CHECK(!any.has_value());
  BOOST_CHECK(any2.has_value());
  BOOST_CHECK(type_id(Type<MoveOnlyType>{}) == any2.type());
  BOOST_CHECK(MoveOnlyType() == any_cast<MoveOnlyType>(any2));

  MoveOnlyType m = any_cast<MoveOnlyType>(std::move(any2));
}
