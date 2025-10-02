#include "blazing/executor.hpp"
#include <optional>

namespace blazing {

// -- Run methods

// executes as soon as motion gets added
void RunExecutor::addMotion(std::unique_ptr<MotionBase> motion) {
    while (true) {
        // std::cout << "evaluating motion!" << std::endl;
        std::optional<motionExecutionResult> result = motion->execute();

        auto result_finished = [](auto result) -> std::optional<bool> {
            return result.finished;
        };

        // finished motion, stop
        if (result.and_then(result_finished).value_or(false)) {
            break;
        }

        pros::delay(motion->getLoopDelayTime());
    }
}

// -- Async methods

// executes as soon as motion gets added
void AsyncExecutor::addMotion(std::unique_ptr<MotionBase> motion) {
    // wait until the queue is available
    std::lock_guard lock(mutex);
    motions.push(std::move(motion));
}

void AsyncExecutor::update() {
    // there is a motion to perform
    mutex.take();

    if (motions.empty()) {
        // delay until a new motion is available
        mutex.give();
        pros::delay(20);
        return;
    }

    std::unique_ptr<MotionBase>& current_motion = motions.front();
    auto result = current_motion->execute();

    mutex.give();

    auto result_finished = [](auto result) -> std::optional<bool> {
        return result.finished;
    };

    if (result.and_then(result_finished).value_or(false)) {
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
void AsyncExecutor::init() {
    pros::Task([this_ptr = this]() {
        // breaks if the executor gets deleted for any reason
        while (this_ptr != nullptr) {
            this_ptr->update();
        }
    });
}

// blocks until all the motions in the queue have finished
void AsyncExecutor::wait() {
    while (!motions.empty()) {
        pros::delay(20);
    }
}

void AsyncExecutor::exitCurrent() {
    std::lock_guard lock(mutex);
    motions.pop();
}

void AsyncExecutor::exitAll() {
    std::lock_guard lock(mutex);
    while (!motions.empty()) {
        motions.pop();
    }
}

void AsyncExecutor::waitUntil(std::function<bool()> condition) {
    while (!condition()) {
        pros::delay(20);
    }
    exitAll();
}

// -- Chained methods

ChainedExecutor::ChainedExecutor(Time fusing_time)
    : default_fusing_duration(fusing_time) {}

ChainedExecutor::ChainedExecutor(
  Time default_fuse_duration,
  std::function<Voltage(Voltage, Voltage, double)> custom_chain_interpolation)
    : default_fusing_duration(default_fuse_duration),
      chain_interpolation(custom_chain_interpolation) {}

// executes as soon as motion gets added
void ChainedExecutor::addMotion(std::unique_ptr<MotionBase> motion) {
    // wait until the queue is available
    std::lock_guard lock(mutex);
    motions.push_back(std::move(motion));
}

void ChainedExecutor::update() {
    // there is a motion to perform
    mutex.take();

    if (motions.empty()) {
        // delay until a new motion is available
		mutex.give();
        pros::delay(20);
        return;
    }

    std::unique_ptr<MotionBase>& current_motion = motions.front();

    // attempt to disable the drivetrain
    bool disabled_result = current_motion->setEnabledDrivetrain(false);

    // run motion logic
    std::optional<motionExecutionResult> result = current_motion->execute();

    auto result_inTolerance =
      [](motionExecutionResult result) -> std::optional<bool> {
        return result.inSmallTolerance.value_or(false) ||
               result.inLargeTolerance.value_or(false) ||
               result.inChainTolerance.value_or(false);
    };

    // starts fusing if any tolerance gets hit
    if (!fuse_start_time.has_value() &&
        result.and_then(result_inTolerance).value_or(false)) {
        fuse_start_time = now();
        std::cout << "start fusing!" << std::endl;
    }

    // get current voltages
    std::optional<std::vector<Voltage>> current_voltages =
      current_motion->getVoltagesDrivetrain();

    bool fusing_finished = false;

    std::cout << "cant fuse: " << disabled_result << std::endl;

    if (motions.size() >= 2 && fuse_start_time &&
        // makes sure we actually disabled the drivetrain
        disabled_result) {
        std::unique_ptr<MotionBase>& next_motion = *next(motions.begin());

        // NOTE: disables next drivetrain in case its different from the one of
        // the current motion (this should never really happen)
        next_motion->setEnabledDrivetrain(false);

        // compute next motion (dont care about its results?)
        next_motion->execute();

        // get its voltages
        std::optional<std::vector<Voltage>> next_voltages =
          next_motion->getVoltagesDrivetrain();

        if (current_voltages && next_voltages &&
            current_voltages->size() == next_voltages->size()) {

            std::vector<Voltage> fused_voltages(current_voltages->size());

            Time elapsed_time = now() - *fuse_start_time;

            // use custom chain time from next motion if specified
            Time fusing_duration =
              next_motion->getChainTime().value_or(default_fusing_duration);

            double normalized_time =
              units::clamp(elapsed_time / fusing_duration, 0.0, 1.0);

            std::cout << "fusing " << normalized_time << std::endl;

            for (size_t i = 0; i < current_voltages->size(); i++) {
                // interpolates between the two voltages
                fused_voltages[i] = chain_interpolation(current_voltages->at(i),
                                                        next_voltages->at(i),
                                                        normalized_time);
            }

            // move drivetrain based on these fused voltages
            current_motion->setEnabledDrivetrain(true);
            current_motion->moveVoltagesDrivetrain(fused_voltages);

            // if we have spent enough time fusing, then just finish the
            // previous motion
            fusing_finished = elapsed_time > fusing_duration;
        } else {
            // mismatch in drivetrains, just perform the current one as usual
            current_motion->setEnabledDrivetrain(true);
            current_motion->moveVoltagesDrivetrain(*current_voltages);
        }
    } else {
        // perform everything current motion normally
        current_motion->setEnabledDrivetrain(true);
        current_motion->moveVoltagesDrivetrain(*current_voltages);
    }

    // not using the queue anymore
    mutex.give();

    auto result_finished = [](auto result) -> std::optional<bool> {
        return result.finished;
    };

    if (fusing_finished || result.and_then(result_finished).value_or(false)) {
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
void ChainedExecutor::init() {
    pros::Task([this_ptr = this]() {
        // breaks if the executor gets deleted for any reason
        while (this_ptr != nullptr) {
            this_ptr->update();
        }
    });
}

// blocks until all the motions in the queue have finished
void ChainedExecutor::wait() {
    while (!motions.empty()) {
        pros::delay(20);
    }
}

void ChainedExecutor::exitCurrent() {
    std::lock_guard lock(mutex);
    motions.pop_front();
}

void ChainedExecutor::exitAll() {
    std::lock_guard lock(mutex);
    motions.clear();
}

void ChainedExecutor::waitUntil(std::function<bool()> condition) {
    while (!condition()) {
        pros::delay(20);
    }
    exitAll();
}
} // namespace blazing
