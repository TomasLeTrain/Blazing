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

            pros::delay(10);
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
    std::optional<Time> fuse_start_time = std::nullopt;
    Time fusing_time;

  protected:
    pros::Mutex mutex;

  public:
    ChainedExecutor(Time fusing_time)
        : fusing_time(fusing_time) {}

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

        // attempt to disable the drivetrain
        bool disabled_result = current_motion->setEnabledDrivetrain(false);

        // run motion logic
        auto result = current_motion->execute();

        // starts fusing if any tolerance gets hit
        if ((result.inSmallTolerance || result.inLargeTolerance) &&
            !fuse_start_time) {
            fuse_start_time = from_msec(pros::millis());
            std::cout << "start fusing!" << std::endl;
        }

        // get current voltages
        std::optional<std::vector<Voltage>> current_voltages =
          current_motion->getVoltagesDrivetrain();

        bool fusing_finished = false;

		std::cout << "cant fuse: " << disabled_result << std::endl;

        if (motions.size() >= 2 && fuse_start_time &&
            // makes sure we can actually disable the drivetrain
            disabled_result) {
            std::unique_ptr<MotionBase>& next_motion = *next(motions.begin());

            // NOTE: we must disable the next motion as well since it likely has
            // a different drivetrain (motions tend to have own all the objects
            // including the drivetrains)
            next_motion->setEnabledDrivetrain(false);

            // compute next motion
            next_motion->execute();

            // get its voltages
            std::optional<std::vector<Voltage>> next_voltages =
              next_motion->getVoltagesDrivetrain();

            if (current_voltages && next_voltages &&
                current_voltages->size() == next_voltages->size()) {

                std::vector<Voltage> fused_voltages(current_voltages->size());

                Time elapsed_time =
                  from_msec(pros::millis()) - *fuse_start_time;
                double normalized_time =
                  units::clamp(elapsed_time / fusing_time, 0.0, 1.0);
				std::cout << "fusing " << normalized_time << std::endl;

                for (size_t i = 0; i < current_voltages->size(); i++) {
                    // fuses between voltages with a simple lerp function
                    fused_voltages[i] =
                      (1 - normalized_time) * current_voltages->at(i) +
                      normalized_time * next_voltages->at(i);
                }

                // move drivetrain based on these fused voltages
                current_motion->setEnabledDrivetrain(true);
                current_motion->moveVoltagesDrivetrain(fused_voltages);

                // if we have spent enough time fusing, then just finish the
                // previous motion
                fusing_finished = elapsed_time > fusing_time;
            } else if (current_voltages.has_value()) {
                // couldn't get the next voltages, just use the current ones
                current_motion->setEnabledDrivetrain(true);
                bool set_voltage_result =
                  current_motion->moveVoltagesDrivetrain(*current_voltages);
            } // else can't do anything since we don't know the voltages
        } else {
            // perform everything as usual
            if (disabled_result && current_voltages) {
                current_motion->setEnabledDrivetrain(true);
                current_motion->moveVoltagesDrivetrain(*current_voltages);
            }
            // else the drivetrain was either never disabled or we don't know
            // the voltages to use either way we don't do anything
        }

        // not using the queue anymore
        mutex.give();

        if (result.finished || fusing_finished) {
            // remove motion from queue, need to take the mutex again
			std::cout << "finished motion " << fusing_finished << std::endl;
            mutex.take();
            motions.pop_front();
            mutex.give();
            fuse_start_time = std::nullopt;

            pros::delay(10);
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
