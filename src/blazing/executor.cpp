#include "blazing/executor.hpp"

namespace blazing {

// -- Run methods

// executes as soon as motion gets added
void RunExecutor::addMotion(std::unique_ptr<MotionBase> motion) {
    while (true) {
        // std::cout << "evaluating motion!" << std::endl;
        auto result = motion->execute();

        // finished motion, stop
        if (result.finished) {
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

void AsyncExecutor::runUntil(std::function<bool()> condition) {
    while (!condition()) {
        pros::delay(20);
    }
    exitAll();
}

// -- Chained methods

ChainedExecutor::ChainedExecutor(Time fusing_time)
    : fusing_time(fusing_time) {}

ChainedExecutor::ChainedExecutor(
  Time fusing_time,
  std::function<Voltage(Voltage, Voltage, double)> custom_chain_interpolation)
    : fusing_time(fusing_time),
      chain_interpolation(custom_chain_interpolation) {}

// executes as soon as motion gets added
void ChainedExecutor::addMotion(std::unique_ptr<MotionBase> motion) {
    // wait until the queue is available
    std::lock_guard lock(mutex);
    motions.push_back(std::move(motion));
}

void ChainedExecutor::update() {
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
    if ((result.inSmallTolerance.value_or(false) ||
         result.inLargeTolerance.value_or(false)) &&
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
        // makes sure we actually disabled the drivetrain
        disabled_result) {
        std::unique_ptr<MotionBase>& next_motion = *next(motions.begin());

        // NOTE: disables next drivetrain in case its different from the one of
        // the current motion (this should never really happen)
        next_motion->setEnabledDrivetrain(false);

        // compute next motion
        next_motion->execute();

        // get its voltages
        std::optional<std::vector<Voltage>> next_voltages =
          next_motion->getVoltagesDrivetrain();

        if (current_voltages && next_voltages &&
            current_voltages->size() == next_voltages->size()) {

            std::vector<Voltage> fused_voltages(current_voltages->size());

            Time elapsed_time = from_msec(pros::millis()) - *fuse_start_time;

            // use custom chain time from next motion if specified
            Time fusing_duration = next_motion->getChainTime() ?
                                     *next_motion->getChainTime() :
                                     fusing_time;

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
            fusing_finished = elapsed_time > fusing_time;
        } else if (current_voltages) {
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
        // else the drivetrain was either never disabled or we don't
        // know the voltages to use either way we don't do anything
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

void ChainedExecutor::runUntil(std::function<bool()> condition) {
    while (!condition()) {
        pros::delay(20);
    }
    exitAll();
}
} // namespace blazing
