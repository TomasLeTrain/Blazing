#pragma once

#include "blazing/motion.hpp"
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

// TODO: might be able to fuse the functionality of the executors into one, but
// is that better? (probably since async and chained commands would not have an
// opportunity to run at the same time)

template<typename M>
constexpr void operator|(M&& motion, Executor& executor) {
    // creates a copy of the temporary motion object and creates one owned by
    // the executor
    std::cout << "called operator!" << std::endl;
    executor.addMotion(
      std::move(std::make_unique<std::decay_t<M>>(std::forward<M>(motion))));
}

class RunExecutor : public Executor {
  public:
    RunExecutor() {}

    // executes as soon as motion gets added
    void addMotion(std::unique_ptr<MotionBase> motion) override {
        while (true) {
            std::cout << "evaluating motion!" << std::endl;
            auto result = motion->execute();

            // finished motion, stop
            if (result.finished) {
                std::cout << "finished run motion!" << std::endl;
                break;
            }

            pros::delay(motion->getLoopDelayTime());
        }
    }
};

class AsyncExecutor : public Executor {
  private:
    std::queue<std::unique_ptr<MotionBase>> motions;

  protected:
    pros::Mutex mutex;

  public:
    AsyncExecutor() {}

    // executes as soon as motion gets added
    void addMotion(std::unique_ptr<MotionBase> motion) override {
        // wait until the queue is available
        std::lock_guard lock(mutex);
        motions.push(std::move(motion));
    }

    void update() {
        if (motions.empty()) {
            // delay until a new motion is available
            pros::delay(20);
            return;
        }
        // there is a motion to perform
        mutex.take();

        std::unique_ptr<MotionBase>& current_motion = motions.front();
        auto result = current_motion->execute();

        mutex.give();

        if (result.finished) {
            // remove motion from queue, need to take the mutex again
            mutex.take();
            motions.pop();
            mutex.give();

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

// similar to the async executor, however instead of immediately going from one
// motion to another, it gradually takes the input from two motions and blends
// them to have one smooth motion
class ChainedExecutor : public Executor {
  private:
    std::list<std::unique_ptr<MotionBase>> motions;
	bool fusing = false;

  protected:
    pros::Mutex mutex;

  public:
    ChainedExecutor() {}

    // executes as soon as motion gets added
    void addMotion(std::unique_ptr<MotionBase> motion) override {
        // wait until the queue is available
        std::lock_guard lock(mutex);
        motions.push_back(std::move(motion));
    }

    void update() {
        if (motions.empty()) {
            // delay until a new motion is available
            pros::delay(20);
            return;
        }
        // there is a motion to perform
        mutex.take();

        std::unique_ptr<MotionBase>& current_motion = motions.front();
		current_motion->getDrivetrain()
        auto result = current_motion->execute();

		if(result.inLargeTolerance && !fusing){
			fusing = true;
		}

		if(motions.size() >= 2 && fusing){
			std::unique_ptr<MotionBase>& next_motion = *next(motions.begin()); 
			auto next_result = next_motion->execute();
		}else{
			// perform everything as usual
		}

        mutex.give();

        if (result.finished) {
            // remove motion from queue, need to take the mutex again
            mutex.take();
            motions.pop_front();
            mutex.give();
			fusing = false;

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
