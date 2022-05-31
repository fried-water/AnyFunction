#include "anyf/any_function.h"

#include "sentinal.h"

#include <boost/test/unit_test.hpp>

using namespace anyf;

namespace {

template <typename... Outputs, typename... Inputs>
std::tuple<Outputs...> invoke_with_values(const AnyFunction& func, Inputs... inputs) {
  auto any_values = make_vector<Any>(inputs...);
  auto result = func(any_ptrs(any_values));

  BOOST_CHECK(sizeof...(Outputs) == result.size());

  return apply_range<sizeof...(Outputs)>(
    std::move(result), [](auto&&... anys) { return std::tuple(any_cast<Outputs>(std::move(anys))...); });
}

void void_fp(){};
int simple_fp() { return 1; };
auto tuple_fp() { return std::tuple('a', 1); };

void no_args(){};
void one_arg(int){};
void many_args(std::tuple<>, const std::string, const char&){};

}; // namespace

BOOST_AUTO_TEST_CASE(test_any_function_return_types) {
  const auto void_func = AnyFunction(void_fp);
  const auto single_func = AnyFunction(simple_fp);
  const auto tuple_func = AnyFunction(tuple_fp);

  BOOST_CHECK(make_types(TypeList<>{}) == void_func.output_types());
  BOOST_CHECK(make_types(TypeList<int>{}) == single_func.output_types());
  BOOST_CHECK(make_types(TypeList<char, int>{}) == tuple_func.output_types());

  BOOST_CHECK(std::tuple() == invoke_with_values(void_func));
  BOOST_CHECK(std::tuple(1) == invoke_with_values<int>(single_func));
  BOOST_CHECK(std::tuple('a', 1) == (invoke_with_values<char, int>(tuple_func)));
}

BOOST_AUTO_TEST_CASE(test_any_function_input_types) {
  const auto no_args_func = AnyFunction(no_args);
  const auto one_arg_func = AnyFunction(one_arg);
  const auto many_args_func = AnyFunction(many_args);

  BOOST_CHECK(make_types(TypeList<>{}) == no_args_func.input_types());
  BOOST_CHECK(make_types(TypeList<int>{}) == one_arg_func.input_types());
  BOOST_CHECK(make_types(TypeList<std::tuple<>, std::string, const char&>{}) ==
    many_args_func.input_types());
}

bool valid_exception(BadCast const&) { return true; }
bool valid_exception(BadInvocation const&) { return true; }

BOOST_AUTO_TEST_CASE(test_any_function_incorrect_args) {
  const auto func = AnyFunction([](int) {});

  BOOST_CHECK_EXCEPTION(invoke_with_values(func, 'a'), BadInvocation, valid_exception);
  BOOST_CHECK_EXCEPTION(invoke_with_values(func), BadInvocation, valid_exception);
  BOOST_CHECK_EXCEPTION(invoke_with_values(func, 1, 2), BadInvocation, valid_exception);

  invoke_with_values(func, 1);
}

BOOST_AUTO_TEST_CASE(test_any_function_no_copies) {
  const auto func = AnyFunction([s = Sentinal{}]() { return s.copies; });

  BOOST_CHECK_EQUAL(0, std::get<0>(invoke_with_values<int>(func)));
  BOOST_CHECK_EQUAL(0, std::get<0>(invoke_with_values<int>(func)));
}

BOOST_AUTO_TEST_CASE(test_any_function_invalid) {
  // auto no_pointers = AnyFunction([](int*){});
  // auto no_const_pointers = AnyFunction([](int const*){});
  // auto no_non_const_refs = AnyFunction([](int&){});

  // auto no_return_refs = AnyFunction([](int const& x) -> int const& { return x; });
  // auto no_return_ptrs = AnyFunction([](int const& x) { return &x; });

  // auto must_be_const = AnyFunction([]() mutable {});
}

BOOST_AUTO_TEST_CASE(test_any_function_num_moves_copies) {
  const auto sentinal_func = AnyFunction([](Sentinal x, const Sentinal& y) {
    BOOST_CHECK_EQUAL(0, x.copies);
    BOOST_CHECK_EQUAL(2, x.moves); // 1 move into any, 1 into function

    BOOST_CHECK_EQUAL(0, y.copies);
    BOOST_CHECK_EQUAL(1, y.moves); // 1 move into any

    return Sentinal{};
  });

  auto input_vals = make_vector<Any>(Sentinal{}, Sentinal{});

  auto result = sentinal_func(any_ptrs(input_vals));

  BOOST_CHECK_EQUAL(1u, result.size());

  const Sentinal* result_sentinal = any_cast<Sentinal>(&result[0]);

  BOOST_CHECK_EQUAL(0, result_sentinal->copies);
  BOOST_CHECK_EQUAL(1, result_sentinal->moves); // 1 move out of function
}

BOOST_AUTO_TEST_CASE(test_any_function_fwd) {
  const auto sentinal_func = AnyFunction([](Sentinal&& x) -> Sentinal&& {
    return std::move(x);
  });

  auto input_vals = make_vector<Any>(Sentinal{});
  auto result = sentinal_func(any_ptrs(input_vals));

  BOOST_CHECK_EQUAL(1u, result.size());

  const Sentinal* result_sentinal = any_cast<Sentinal>(&result[0]);

  BOOST_CHECK_EQUAL(0, result_sentinal->copies);
  BOOST_CHECK_EQUAL(2, result_sentinal->moves); // 1 move into any, 1 move into result
}
