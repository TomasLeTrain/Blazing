#include "blazing/executor.hpp"
#include "pros/misc.hpp"
#include <mutex>
#include <optional>

namespace blazing {

// -- Run methods

// executes as soon as motion gets added
void RunExecutor::addMotion(std::unique_ptr<MotionBase> motion) {
    auto m_originalCompStatus = pros::competition::get_status();

    bool finished_halfway = false;

    motion->start_motion_callback();

    while (true) {
        if (pros::competition::get_status() != m_originalCompStatus) {
            // should break out of motion
            finished_halfway = true;
            break;
        }

        uint32_t start_time = pros::millis();

        std::optional<motionExecutionResult> result = motion->execute();

        auto result_finished = [](auto result) -> std::optional<bool> {
            return result.finished;
        };

        // finished motion, stop
        if (result.and_then(result_finished).value_or(false)) {
            break;
        }

        pros::c::task_delay_until(&start_time, motion->getLoopDelayTime());
    }

    motion->end_motion_callback();
}


// Some async methods used for both AsyncExecutor and ChainedExecutor

// start the async task
void AsyncExecutorBase::init() {
    pros::Task([this_ptr = this]() {
        // breaks if the executor gets deleted for any reason
        while (this_ptr != nullptr) {
            this_ptr->update();
        }
    });
}

bool AsyncExecutorBase::hasMotions() {
    return this->numQueuedMotions() > 0;
}

void AsyncExecutorBase::exitAll() {
    std::lock_guard lock(m_mutex);
    while (hasMotions()) {
        exitCurrent();
    }
}

// blocks until all the motions in the queue have finished
void AsyncExecutorBase::wait() {
    while (hasMotions()) {
        pros::delay(20);
    }
}

void AsyncExecutorBase::waitUntil(std::function<bool()> condition) {
    // if condition is true or has motions is false it breaks
    while (!condition() && hasMotions()) {
        pros::delay(20);
    }
}

void AsyncExecutorBase::stopIf(std::function<bool()> condition) {
    waitUntil(condition);
    exitAll();
}

size_t AsyncExecutorBase::getCurrentIndex() {
    return latest_motion_index;
}

size_t AsyncExecutorBase::getFinishedIndex() {
    return finished_index;
}

// waits until the finished index matches the given index
void AsyncExecutorBase::waitUntilIndex(size_t index) {
    this->waitUntil([this, index] {
        return index == this->getFinishedIndex();
    });
}

// AsyncExecutor methods

// executes as soon as motion gets added
void AsyncExecutor::addMotion(std::unique_ptr<MotionBase> motion) {
    // wait until the queue is available
    std::lock_guard lock(m_mutex);

    // in case a motion is added in a different competition state
    if (pros::competition::get_status() != m_currentCompStatus) {
        // should break out of all motions
        exitAll();
        m_currentCompStatus = pros::competition::get_status();
    }

    motions.push(std::move(motion));

    latest_motion_index++;
}

void AsyncExecutor::update() {
    uint32_t start_time = pros::millis();
    uint32_t delay_time = 10;

    // mutex is taken care of automatically in this scope
    {
        std::lock_guard lock(m_mutex);
        if (pros::competition::get_status() != m_currentCompStatus) {
            // should break out of all motions
            exitAll();
            m_currentCompStatus = pros::competition::get_status();
        }

        if (!hasMotions()) {
            delay_time = 20;
            goto endupdate;
        }

        std::unique_ptr<MotionBase>& current_motion = motions.front();

        if (start_of_motion) {
            current_motion->start_motion_callback();
            start_of_motion = false;
        }

        delay_time = current_motion->getLoopDelayTime();
        std::optional<motionExecutionResult> result = current_motion->execute();

        auto result_finished = [](auto result) -> std::optional<bool> {
            return result.finished;
        };

        if (result.and_then(result_finished).value_or(false)) {
			motions.front()->end_motion_callback();
            exitCurrent();

            // don't sleep to execute next motion immediately
            delay_time = 1;
        }
    }

    // delay until next update
endupdate:
    pros::c::task_delay_until(&start_time, delay_time);
}

void AsyncExecutor::exitCurrent() {
    std::lock_guard lock(m_mutex);
    motions.pop();
    start_of_motion = true;
    finished_index++;
}

size_t AsyncExecutor::numQueuedMotions() {
    std::lock_guard lock(m_mutex);
    return motions.size();
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
    std::lock_guard lock(m_mutex);

    // in case a motion is added in a different competition state
    if (pros::competition::get_status() != m_currentCompStatus) {
        // should break out of all motions
        exitAll();
        m_currentCompStatus = pros::competition::get_status();
    }

    motions.push_back(std::move(motion));

    latest_motion_index++;
}

void ChainedExecutor::update() {
    uint32_t start_time = pros::millis();
    uint32_t delay_time = 10;

    // mutex is taken care of automatically in this scope
    {
        std::lock_guard lock(m_mutex);
        if (pros::competition::get_status() != m_currentCompStatus) {
            // should break out of all motions
            exitAll();
            m_currentCompStatus = pros::competition::get_status();
        }

        if (!hasMotions()) {
            // delay until a new motion is available
            delay_time = 20;
            goto endupdate;
        }

        std::unique_ptr<MotionBase>& current_motion = motions.front();

        if (start_of_motion) {
            current_motion->start_motion_callback();
            start_of_motion = false;
        }

        delay_time = current_motion->getLoopDelayTime();

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
        }

        // get current voltages
        std::optional<std::vector<Voltage>> current_voltages =
          current_motion->getVoltagesDrivetrain();

        bool fusing_finished = false;

        if (motions.size() >= 2 && fuse_start_time.has_value() &&
            // makes sure we actually disabled the drivetrain
            disabled_result) {
            std::unique_ptr<MotionBase>& next_motion = *next(motions.begin());

            // NOTE: disables next drivetrain in case its different from the one
            // of the current motion (this should never really happen)
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

                for (size_t i = 0; i < current_voltages->size(); i++) {
                    // interpolates between the two voltages
                    fused_voltages[i] =
                      chain_interpolation(current_voltages->at(i),
                                          next_voltages->at(i),
                                          normalized_time);
                }

                // move drivetrain based on these fused voltages
                current_motion->setEnabledDrivetrain(true);
                current_motion->moveVoltagesDrivetrain(fused_voltages);

                // if we have spent enough time fusing, then just finish the
                // motion
                fusing_finished = elapsed_time > fusing_duration;
            } else {
                // mismatch in drivetrains, just perform the current one as
                // usual
                current_motion->setEnabledDrivetrain(true);
                current_motion->moveVoltagesDrivetrain(*current_voltages);
            }
        } else {
            // perform everything current motion normally
            current_motion->setEnabledDrivetrain(true);
            current_motion->moveVoltagesDrivetrain(*current_voltages);
        }

        auto result_finished = [](auto result) -> std::optional<bool> {
            return result.finished;
        };

        if (fusing_finished ||
            result.and_then(result_finished).value_or(false)) {
            // finish motion
			motions.front()->end_motion_callback();
            exitCurrent();

            // execute next motion immediately
            delay_time = 1;
        }
    }

    // delay until next update
endupdate:
    pros::c::task_delay_until(&start_time, delay_time);
}

void ChainedExecutor::exitCurrent() {
    std::lock_guard lock(m_mutex);
    motions.pop_front();
    start_of_motion = true;
    finished_index++;

    fuse_start_time = std::nullopt;
}

size_t ChainedExecutor::numQueuedMotions() {
    std::lock_guard lock(m_mutex);
    return motions.size();
}

} // namespace blazing
