#pragma once

#include "blazing/motion.hpp"
#include "units/units.hpp"
#include <memory>
#include <queue>
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
            auto result = motion->execute();

            // finished motion, stop
            if (result.finished) {
                break;
            }

            pros::delay(motion->getLoopDelayTime());
        }
    }
};

class AsyncExecutor : public Executor {
  private:
    std::queue<std::unique_ptr<MotionBase>> motions;

  public:
    AsyncExecutor() {}

    // executes as soon as motion gets added
    void addMotion(std::unique_ptr<MotionBase> motion) override {
        motions.push(std::move(motion));
    }

    void update() {
        if (motions.empty()) {
            // delay until a new motion is available
            pros::delay(20);
            return;
        }
        // there is a motion to perform
        std::unique_ptr<MotionBase>& current_motion = motions.front();
        auto result = current_motion->execute();
        if (result.finished) {
            // remove motion from queue
            motions.pop();
            pros::delay(20);
        } else {
            pros::delay(current_motion->getLoopDelayTime());
        }
    }

    // start the async task
    void init() {
        pros::Task([this_ptr = this]() {
            // breaks if the executor gets deleted for any reason
            while (this_ptr != nullptr) {
                this_ptr->update();
            }
        });
    }

    // blocks until all the motions in the queue have finished
    void wait() {
        while (!motions.empty()) {
            pros::delay(20);
        }
    }
};
} // namespace blazing
