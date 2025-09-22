#pragma once

#include "blazing/motions/motion.hpp"
#include "pros/rtos.hpp"
#include "units/units.hpp"
#include <cstddef>
#include <iostream>
#include <list>
#include <memory>
#include <mutex>
#include <optional>
#include <ostream>
#include <queue>
#include <type_traits>

namespace blazing {
class Executor {
  public:
    virtual void addMotion(std::unique_ptr<MotionBase> motion) = 0;
    virtual ~Executor() = default;
};

// TODO: figure out a way to keep the motions from running at the same (maybe
// passing a motion mutex)

template<typename M>
constexpr void operator|(M&& motion, Executor& executor) {
    // creates a copy of the temporary motion object and creates one owned by
    // the executor
	std::cout << "operator called!" << std::endl;
    executor.addMotion(
      std::move(std::make_unique<std::decay_t<M>>(std::forward<M>(motion))));
}

class RunExecutor : public Executor {
  public:
    RunExecutor() {}

    // executes as soon as motion gets added
    void addMotion(std::unique_ptr<MotionBase> motion) override;
};

class AsyncExecutor : public Executor {
  private:
    std::queue<std::unique_ptr<MotionBase>> motions;

  protected:
    pros::Mutex mutex;

  public:
    AsyncExecutor() {}

    // executes as soon as motion gets added
    void addMotion(std::unique_ptr<MotionBase> motion) override;

    void update();

    // start the async task
    void init();

    // blocks until all the motions in the queue have finished
    void wait();

    // moves on to the next motion immediately
    void exitCurrent();

    // clears all motions that were gonna be executed from queue
    void exitAll();

    // waits until the function returns true, after which it exists all queued
    // motions
    void waitUntil(std::function<bool()> condition);
};

struct ChainOptions {
    std::optional<Time> fuse_start_time = std::nullopt;
};

// similar to the async executor, however instead of immediately going from one
// motion to another, it gradually takes the input from two motions and blends
// them to have one smooth motion
class ChainedExecutor : public Executor {
  private:
    std::list<std::unique_ptr<MotionBase>> motions;
    std::optional<Time> fuse_start_time = std::nullopt;
    Time default_fusing_duration;

    std::function<Voltage(Voltage, Voltage, double)> chain_interpolation =
      [](Voltage a, Voltage b, double t) {
          return (1 - t) * a + t * b;
      };

  protected:
    pros::Mutex mutex;

  public:
    ChainedExecutor(Time fusing_time);

    ChainedExecutor(Time default_fuse_duration,
                    std::function<Voltage(Voltage, Voltage, double)>
                      custom_chain_interpolation);

    // executes as soon as motion gets added
    void addMotion(std::unique_ptr<MotionBase> motion) override;

    void update();

    // start the async task
    void init();

    // blocks until all the motions in the queue have finished
    void wait();

    // exit current motion, moves onto next motion immediately
    void exitCurrent();

    // exits all motions that were gonna be executed from queue
    void exitAll();

    // waits until the function returns true, after which it exists all queued
    // motions
    void waitUntil(std::function<bool()> condition);
};
} // namespace blazing
