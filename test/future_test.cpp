#include "anyf/future.h"

#include <thread>

#include <boost/test/unit_test.hpp>

using namespace anyf;

namespace {

struct SeqExecutor {
  template <typename F>
  void operator()(F&& f) const { std::forward<F>(f)(); }
};

}

BOOST_AUTO_TEST_CASE(promise_future_cleanup) {
  auto [p, f] = make_promise_future();

  p.~Promise();
  f.~Future();
}

BOOST_AUTO_TEST_CASE(future_promise_cleanup) {
  auto [p, f] = make_promise_future();

  f.~Future();
  p.~Promise();
}

BOOST_AUTO_TEST_CASE(future_wait_no_send) {
  auto [p, f] = make_promise_future();

  p.~Promise();

  BOOST_CHECK(!std::move(f).wait().has_value());
}

BOOST_AUTO_TEST_CASE(future_then_no_send) {
  auto [p, f] = make_promise_future();

  std::move(f).then(SeqExecutor{}, [&](Any v) {
    BOOST_CHECK(!v.has_value());
  });
}

BOOST_AUTO_TEST_CASE(future_no_send_then) {
  auto [p, f] = make_promise_future();

  p.~Promise();

  std::move(f).then(SeqExecutor{}, [&](Any v) {
    BOOST_CHECK(!v.has_value());
  });
}

BOOST_AUTO_TEST_CASE(future_send_no_receive) {
  auto [p, f] = make_promise_future();

  std::move(p).send(SeqExecutor{}, 1);
}

BOOST_AUTO_TEST_CASE(future_no_receive_send) {
  auto [p, f] = make_promise_future();

  f.~Future();

  std::move(p).send(SeqExecutor{}, 1);
}

BOOST_AUTO_TEST_CASE(future_send_wait) {
  auto [p, f] = make_promise_future();

  std::move(p).send(SeqExecutor{}, 1);

  BOOST_CHECK_EQUAL(1, any_cast<int>(std::move(f).wait()));
}

BOOST_AUTO_TEST_CASE(future_send_then) {
  auto [p, f] = make_promise_future();

  std::move(p).send(SeqExecutor{}, 1);
  std::move(f).then(SeqExecutor{}, [](Any v) {
    BOOST_CHECK_EQUAL(1, any_cast<int>(v));
  });
}

BOOST_AUTO_TEST_CASE(future_then_send) {
  auto [p, f] = make_promise_future();

  std::move(f).then(SeqExecutor{}, [](Any v) {
    BOOST_CHECK_EQUAL(1, any_cast<int>(v));
  });
  std::move(p).send(SeqExecutor{}, 1);
}

BOOST_AUTO_TEST_CASE(future_wait_stress) {
  constexpr int count = 1000;

  std::atomic<int> calls = 0;

  std::vector<std::thread> threads;

  for(int i = 0; i < count; i++) {
    auto [p, f] = make_promise_future();
    threads.emplace_back([p = std::move(p)]() mutable { std::move(p).send(SeqExecutor{}, 1); });
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
    auto [p, f] = make_promise_future();
    threads.emplace_back([p = std::move(p)]() mutable { std::move(p).send(SeqExecutor{}, 1); });
    threads.emplace_back([f = std::move(f), &calls]() mutable {
      std::move(f).then(SeqExecutor{}, [&](Any v) {
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
