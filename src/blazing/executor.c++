#include "blazing/executor.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/rtos.h"
#include <functional>
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
        // if being executed on main task (i.e. autonomous or opcontrol) this if
        // statement would never be reached; the task would get deleted before
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

class RunWithAsync {
  public:
    std::reference_wrapper<AsyncExecutor> m_async_executor;

    RunWithAsync(std::reference_wrapper<AsyncExecutor> async_executor)
        : m_async_executor(async_executor) {}

    void addMotion(std::unique_ptr<MotionBase> motion) {
        m_async_executor.get().wait();
        m_async_executor.get().addMotion(std::move(motion));
        m_async_executor.get().wait();
    }
};

// Some async methods used for both AsyncExecutor and ChainedExecutor

// start the async task
void AsyncExecutorBase::init() {
    pros::Task(
      [this_ptr = this]() {
          // breaks if the executor gets deleted for any reason
          while (this_ptr != nullptr) {
              this_ptr->update();
          }
      },
      TASK_PRIORITY_DEFAULT,
      // give them a lot of memory
      TASK_STACK_DEPTH_DEFAULT * 2);
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

// waits until condition triggers or no motions are left
// returns true if all motions finished before condition.
// also has optional timeout
AsyncExecutorBase::waitOrT
AsyncExecutorBase::waitOr(std::function<bool()> condition,
                          std::optional<Time> timeout) {
    bool condition_met, motion_met, timeout_met;
    Time start_time = now();
    while (true) {
        condition_met = condition();
        motion_met = numQueuedMotions() == 0;
        timeout_met = blazing::timeoutDone(timeout, start_time);

        if (condition_met || motion_met || timeout_met) break;
        pros::delay(10);
    }

    if (condition_met)
        return conditionFinished;
    else if (timeout_met)
        return timeoutFinished;
    else if (motion_met)
        return motionFinished;
    else
        // something went wrong, just assume all motions finished?
        return motionFinished;
};

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

void AsyncExecutorBase::checkCompStatus() {
    std::lock_guard lock(m_mutex);

    if (!m_currentCompStatus.has_value())
        m_currentCompStatus = pros::competition::get_status();

    if (pros::competition::get_status() != m_currentCompStatus.value()) {
        // should break out of all motions
        exitAll();
        m_currentCompStatus = pros::competition::get_status();
    }
}

std::optional<std::uint8_t> AsyncExecutorBase::getLatestCompStatus() {
    return m_currentCompStatus;
}

// AsyncExecutor methods

void AsyncExecutor::setCustomExitCondition(
  CustomExitConditionT custom_exit_condition) {
    m_custom_exit_condition = custom_exit_condition;
}

// executes as soon as motion gets added
void AsyncExecutor::addMotion(std::unique_ptr<MotionBase> motion) {
    // wait until the queue is available
    std::lock_guard lock(m_mutex);

    checkCompStatus();

    motions.push(std::move(motion));

    latest_motion_index++;
}

void AsyncExecutor::update() {
    uint32_t start_time = pros::millis();
    uint32_t delay_time = 10;

    // mutex is taken care of automatically in this scope
    {
        std::lock_guard lock(m_mutex);

        checkCompStatus();

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

        auto result_finished =
          [this](motionExecutionResult result) -> std::optional<bool> {
            return result.finished || m_custom_exit_condition(result, this);
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

// ChainedExecutor::ChainedExecutor(Time fusing_time)
//     : default_fusing_duration(fusing_time) {}
//
// ChainedExecutor::ChainedExecutor(
//   Time default_fuse_duration,
//   std::function<Voltage(Voltage, Voltage, double)>
//   custom_chain_interpolation)
//     : default_fusing_duration(default_fuse_duration),
//       chain_interpolation(custom_chain_interpolation) {}
//
// // executes as soon as motion gets added
// void ChainedExecutor::addMotion(std::unique_ptr<MotionBase> motion) {
//     // wait until the queue is available
//     std::lock_guard lock(m_mutex);
//
//     checkCompStatus();
//
//     motions.push(std::move(motion));
//
//     latest_motion_index++;
// }
//
// void ChainedExecutor::update() {
//     uint32_t start_time = pros::millis();
//     uint32_t delay_time = 10;
//
//     // mutex is taken care of automatically in this scope
//     {
//         std::lock_guard lock(m_mutex);
//
//         checkCompStatus();
//
//         if (!hasMotions()) {
//             delay_time = 20;
//             goto endupdate;
//         }
//
//         std::unique_ptr<MotionBase>& current_motion = motions.front();
//
//         if (start_of_motion) {
//             current_motion->start_motion_callback();
//             start_of_motion = false;
//         }
//
//         delay_time = current_motion->getLoopDelayTime();
//         std::optional<motionExecutionResult> result =
//         current_motion->execute();
//
//         auto result_finished = [](auto result) -> std::optional<bool> {
//             return result.finished ||
//             result.inChainTolerance.value_or(false);
//         };
//
//         auto custom_exit = [](auto result) -> std::optional<bool> {
//             return result.inChainTolerance.value_or(false);
//         };
//
//         std::function<bool(std::optional<motionExecutionResult>)> thing;
//
//         auto result_finished2 = [thing](auto result) -> std::optional<bool> {
//             return result.finished || thing(result);
//         };
//
//         if (result.and_then(result_finished).value_or(false)) {
//             motions.front()->end_motion_callback();
//             exitCurrent();
//
//             // don't sleep to execute next motion immediately
//             delay_time = 1;
//         }
//     }
//
//     // delay until next update
// endupdate:
//     pros::c::task_delay_until(&start_time, delay_time);
// }
//
// void ChainedExecutor::exitCurrent() {
//     std::lock_guard lock(m_mutex);
//     motions.pop();
//     start_of_motion = true;
//     finished_index++;
// }
//
// size_t ChainedExecutor::numQueuedMotions() {
//     std::lock_guard lock(m_mutex);
//     return motions.size();
// }

} // namespace blazing
