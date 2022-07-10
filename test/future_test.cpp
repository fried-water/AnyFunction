#include "anyf/future.h"

#include "anyf/executor/sequential_executor.h"

#include <thread>

#include <boost/test/unit_test.hpp>

using namespace anyf;

static const SequentialExecutor executor;

BOOST_AUTO_TEST_CASE(future_promise_cleanup) {
  auto [p, f] = make_promise_future(executor);
}

BOOST_AUTO_TEST_CASE(promise_future_cleanup) {
  auto [p, f] = make_promise_future(executor);
  p = {};
}

BOOST_AUTO_TEST_CASE(future_wait_no_send) {
  auto [p, f] = make_promise_future(executor);

  p = {};

  BOOST_CHECK(!std::move(f).wait().has_value());
}

BOOST_AUTO_TEST_CASE(future_then_no_send) {
  auto [p, f] = make_promise_future(executor);

  std::move(f).then([&](Any v) {
    BOOST_CHECK(!v.has_value());
  });
}

BOOST_AUTO_TEST_CASE(future_no_send_then) {
  auto [p, f] = make_promise_future(executor);

  p = {};

  std::move(f).then([&](Any v) {
    BOOST_CHECK(!v.has_value());
  });
}

BOOST_AUTO_TEST_CASE(future_send_no_receive) {
  auto [p, f] = make_promise_future(executor);

  std::move(p).send(1);
}

BOOST_AUTO_TEST_CASE(future_no_receive_send) {
  auto [p, f] = make_promise_future(executor);

  f = {};

  std::move(p).send(1);
}

BOOST_AUTO_TEST_CASE(future_send_wait) {
  auto [p, f] = make_promise_future(executor);

  std::move(p).send(1);

  BOOST_CHECK_EQUAL(1, any_cast<int>(std::move(f).wait()));
}

BOOST_AUTO_TEST_CASE(future_send_then) {
  auto [p, f] = make_promise_future(executor);

  std::move(p).send(1);
  std::move(f).then([](Any v) {
    BOOST_CHECK_EQUAL(1, any_cast<int>(v));
  });
}

BOOST_AUTO_TEST_CASE(future_then_send) {
  auto [p, f] = make_promise_future(executor);

  std::move(f).then([](Any v) {
    BOOST_CHECK_EQUAL(1, any_cast<int>(v));
  });
  std::move(p).send(1);
}

BOOST_AUTO_TEST_CASE(future_then_then_send) {
  auto [p, f] = make_promise_future(executor);

  std::move(f).then([](Any v) {
    BOOST_CHECK_EQUAL(1, any_cast<int>(v));
    return 2;
  }).then([](Any v) {
    BOOST_CHECK_EQUAL(2, any_cast<int>(v));
  });

  std::move(p).send(1);
}

BOOST_AUTO_TEST_CASE(future_send_then_then_wait) {
  auto [p, f] = make_promise_future(executor);

  std::move(p).send(1);

  const Any result = std::move(f).then([](Any v) {
    BOOST_CHECK_EQUAL(1, any_cast<int>(v));
    return 2;
  }).then([](Any v) {
    BOOST_CHECK_EQUAL(2, any_cast<int>(v));
    return 3;
  }).wait();

  BOOST_CHECK_EQUAL(3, any_cast<int>(result));
}

BOOST_AUTO_TEST_CASE(future_wait_stress) {
  constexpr int count = 1000;

  std::atomic<int> calls = 0;

  std::vector<std::thread> threads;

  for(int i = 0; i < count; i++) {
    auto [p, f] = make_promise_future(executor);
    threads.emplace_back([p = std::move(p)]() mutable { std::move(p).send(1); });
    threads.emplace_back([f = std::move(f), &calls]() mutable { 
      BOOST_CHECK_EQUAL(1, any_cast<int>(std::move(f).wait()));
      calls++;
    });
  }

  for(auto& thread : threads) {
    thread.join();
  }

  BOOST_CHECK_EQUAL(count, calls.load());
}

BOOST_AUTO_TEST_CASE(future_then_stress) {
  constexpr int count = 1000;

  std::atomic<int> calls = 0;

  std::vector<std::thread> threads;

  for(int i = 0; i < count; i++) {
    auto [p, f] = make_promise_future(executor);
    threads.emplace_back([p = std::move(p)]() mutable { std::move(p).send(1); });
    threads.emplace_back([f = std::move(f), &calls]() mutable {
      std::move(f).then([&](Any v) {
        BOOST_CHECK_EQUAL(1, any_cast<int>(v));
        calls++;
      });
    });
  }

  for(auto& thread : threads) {
    thread.join();
  }

  BOOST_CHECK_EQUAL(count, calls.load());
}
