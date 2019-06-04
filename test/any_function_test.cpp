#include "any_function.h"

#include "sentinal.h"

#define BOOST_TEST_DYN_LINK
#include <boost/test/unit_test.hpp>

#include <boost/range/algorithm/equal.hpp>

using namespace anyf;

namespace {

template <typename... Outputs, typename... Inputs>
std::tuple<Outputs...> invoke_with_values(AnyFunction func, Inputs... inputs) {
  auto any_vec = util::make_small_vector<std::any, 3>(inputs...);

  auto result = func(util::map<AnyFunction::InvokeInput>(any_vec, [](auto& x) { return &x; }));

  BOOST_CHECK_EQUAL(sizeof...(Outputs), result.size());

  return util::vec_to_tuple<std::tuple<Outputs...>>(std::move(result));
}

void void_fp(){};
int simple_fp() { return 1; };
auto tuple_fp() { return std::make_tuple('a', 1); };

void no_args(){};
void one_arg(int){};
void many_args(std::tuple<>, const std::string, char const&){};

}; // namespace

BOOST_AUTO_TEST_CASE(test_any_function_return_types) {
  auto void_func = AnyFunction(void_fp);
  auto single_func = AnyFunction(simple_fp);
  auto tuple_func = AnyFunction(tuple_fp);

  BOOST_CHECK(boost::equal(std::vector<Type>{}, void_func.output_types()));
  BOOST_CHECK(boost::equal(std::vector<Type>{make_type<int>()}, single_func.output_types()));
  BOOST_CHECK(boost::equal(std::vector<Type>{make_type<char>(), make_type<int>()},
                           tuple_func.output_types()));

  BOOST_CHECK(std::make_tuple() == invoke_with_values(void_func));
  BOOST_CHECK(std::make_tuple(1) == invoke_with_values<int>(single_func));
  auto macros_suck = std::make_tuple('a', 1) == invoke_with_values<char, int>(tuple_func);
  BOOST_CHECK(macros_suck);
}

BOOST_AUTO_TEST_CASE(test_any_function_input_types) {
  auto no_args_func = AnyFunction(no_args);
  auto one_arg_func = AnyFunction(one_arg);
  auto many_args_func = AnyFunction(many_args);

  BOOST_CHECK(boost::equal(std::vector<Type>{}, no_args_func.input_types()));
  BOOST_CHECK(boost::equal(std::vector<Type>{make_type<int>()}, one_arg_func.input_types()));
  BOOST_CHECK(boost::equal(std::vector<Type>{make_type<std::tuple<>>(), make_type<std::string>(),
                                             make_type<char const&>()},
                           many_args_func.input_types()));
}

bool valid_exception(std::bad_any_cast const&) { return true; }
bool valid_exception(BadInvocation const&) { return true; }

BOOST_AUTO_TEST_CASE(test_any_function_incorrect_args) {
  auto func = AnyFunction([](int) {});

  BOOST_CHECK_EXCEPTION(invoke_with_values(func, 'a'), std::bad_any_cast, valid_exception);

  BOOST_CHECK_EXCEPTION(invoke_with_values(func), BadInvocation, valid_exception);
  BOOST_CHECK_EXCEPTION(invoke_with_values(func, 1, 2), BadInvocation, valid_exception);
  invoke_with_values(func, 1);
}

BOOST_AUTO_TEST_CASE(test_any_function_invalid) {
  // auto no_pointers = AnyFunction([](int*){});
  // auto no_const_pointers = AnyFunction([](int const*){});
  // auto no_non_const_refs = AnyFunction([](int&){});

  // auto no_return_refs = AnyFunction([](int const& x) -> int const& {
  // return x; }); auto no_return_ptrs = AnyFunction([](int const& x) {
  // return &x; });

  // auto must_be_const = AnyFunction([]() mutable {});
}

BOOST_AUTO_TEST_CASE(test_any_function_num_moves_copies) {
  auto sentinal_func = AnyFunction([](Sentinal x, Sentinal const& y) {
    BOOST_CHECK_EQUAL(0, x.copies);
    BOOST_CHECK_EQUAL(2, x.moves); // 1 move into any, 1 into function

    BOOST_CHECK_EQUAL(0, y.copies);
    BOOST_CHECK_EQUAL(1, y.moves); // 1 move into any

    return Sentinal{};
  });

  auto input_vals = util::make_small_vector<std::any, 3>(Sentinal{}, Sentinal{});

  auto result =
      sentinal_func(util::map<AnyFunction::InvokeInput>(input_vals, [](auto& x) { return &x; }));

  BOOST_CHECK_EQUAL(1u, result.size());

  Sentinal const* result_sentinal = std::any_cast<Sentinal>(&result[0]);

  BOOST_CHECK_EQUAL(0, result_sentinal->copies);
  BOOST_CHECK_EQUAL(1, result_sentinal->moves); // 1 move out of function
}
