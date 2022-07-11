#include "anyf/borrowed_future.h"

#include "anyf/executor/sequential_executor.h"

#include <random>
#include <thread>

#include <boost/test/unit_test.hpp>

using namespace anyf;

static const SequentialExecutor executor;

BOOST_AUTO_TEST_CASE(borrowed_future_cleanup) {
  auto [p, f] = make_promise_future(executor);
  auto [b, f2] = borrow(std::move(f));
}

BOOST_AUTO_TEST_CASE(borrowed_future_cleanup_promise_first) {
  auto [p, f] = make_promise_future(executor);
  auto [b, f2] = borrow(std::move(f));
  p = {};
}

BOOST_AUTO_TEST_CASE(borrowed_future_cleanup_future_first) {
  auto [p, f] = make_promise_future(executor);
  auto [b, f2] = borrow(std::move(f));
  f = {};
}

BOOST_AUTO_TEST_CASE(borrowed_future_copy_cleanup) {
  auto [p, f] = make_promise_future(executor);
  auto [b, f2] = borrow(std::move(f));
  auto b2 = b;
}

BOOST_AUTO_TEST_CASE(borrowed_future_forward) {
  auto [p, f] = make_promise_future(executor);
  auto [b, f2] = borrow(std::move(f));

  std::move(p).send(1);
  b = {};
  BOOST_CHECK_EQUAL(1, any_cast<int>(std::move(f2).wait()));
}

BOOST_AUTO_TEST_CASE(borrowed_future_send_wait) {
  auto [p, f] = make_promise_future(executor);
  auto [b, f2] = borrow(std::move(f));

  auto b2 = b;

  std::move(p).send(1);

  BOOST_CHECK_EQUAL(1, any_cast<int>(b.wait()));
  BOOST_CHECK_EQUAL(1, any_cast<int>(b2.wait()));
}

BOOST_AUTO_TEST_CASE(borrowed_future_send_then) {
  auto [p, f] = make_promise_future(executor);
  auto [b, f2] = borrow(std::move(f));

  auto b2 = b;

  std::move(p).send(1);

  b.then([](Any v) {
    BOOST_CHECK_EQUAL(1, any_cast<int>(v));
  });

  b2.then([](Any v) {
    BOOST_CHECK_EQUAL(1, any_cast<int>(v));
    return 2;
  }).then([](Any v) {
    BOOST_CHECK_EQUAL(2, any_cast<int>(v));
  });
}

BOOST_AUTO_TEST_CASE(borrowed_future_then_send) {
  auto [p, f] = make_promise_future(executor);
  auto [b, f2] = borrow(std::move(f));

  auto b2 = b;

  b.then([](Any v) {
    BOOST_CHECK_EQUAL(1, any_cast<int>(v));
  });

  b2.then([](Any v) {
    BOOST_CHECK_EQUAL(1, any_cast<int>(v));
    return 2;
  }).then([](Any v) {
    BOOST_CHECK_EQUAL(2, any_cast<int>(v));
  });

  std::move(p).send(1);
}


BOOST_AUTO_TEST_CASE(borrowed_future_stress) {
  constexpr int count = 500;

  std::random_device rd;
  std::mt19937 rng(rd());

  std::atomic<int> calls = 0;

  int invocations = 0;

  std::vector<std::function<void()>> functions;
  for(int i = 0; i < count; i++) {
    auto [p, f] = make_promise_future(executor);
    auto [b, f2] = borrow(std::move(f));

    functions.emplace_back([p = std::make_shared<Promise>(std::move(p))]() mutable { std::move(*p).send(1); });
    functions.emplace_back([f2 = std::make_shared<Future>(std::move(f2)), &calls]() mutable { 
      BOOST_CHECK_EQUAL(1, any_cast<int>(std::move(*f2).wait()));
      calls++;
    });

    const int copies = rd() % 4;

    for(int i = 0; i < copies; i++) {
      if(rd() % 2 == 0) {
        functions.emplace_back([b = b, &calls]() mutable { 
          BOOST_CHECK_EQUAL(1, any_cast<int>(b.wait()));
          calls++;
        });
      } else {
        functions.emplace_back([b = b, &calls]() mutable {
          b.then([&](Any v) {
            BOOST_CHECK_EQUAL(1, any_cast<int>(v));
            calls++;
          });
        });
      }
    }
  }

  std::shuffle(functions.begin(), functions.end(), rng);

  std::vector<std::thread> threads;
  for(auto& f : functions) {
    threads.emplace_back(std::move(f));
  }

  for(auto& thread : threads) {
    thread.join();
  }

  BOOST_CHECK_EQUAL(functions.size() - count, calls.load());
}
