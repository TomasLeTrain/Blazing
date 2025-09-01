#pragma once

#include "blazing/motion.hpp"
#include "units/units.hpp"
#include <memory>
#include <type_traits>

namespace blazing {
class Executor {
  public:
    virtual void addMotion(std::unique_ptr<MotionBase> motion) = 0;
    virtual ~Executor() = default;
};

template<typename M>
constexpr void operator|(M&& motion, Executor& executor) {
    // creates a copy of the temporary motion object and creates one owned by
    // the executor
    executor.addMotion(
      std::move(std::make_unique<std::decay_t<M>>(std::forward<M>(motion))));
}

class RunExecutor : public Executor {
  public:
    RunExecutor() {}

    // executes as soon as motion gets added
    void addMotion(std::unique_ptr<MotionBase> motion) override {
        while (true) {
            motionExecutionResult result = motion->execute();

            // finished motion, stop
            if (result.finished) {
                break;
            }

            pros::delay(motion->getLoopDelayTime());
        }
    }
};
} // namespace blazing
