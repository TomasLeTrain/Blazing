#pragma once

#include "pros/rtos.hpp"

namespace blazing {

class Motion {
  protected:
    virtual int getLoopDelayTime() = 0;
    virtual void execute() = 0;

  public:
    Motion() {}

    // waits until finishes
    void run() {
        while (true) {
            this->execute();
            pros::delay(getLoopDelayTime());
        }
    };

    // runs async
    void async() {
        // spawn a task to run this in
        pros::Task([this]() {
            this->run();
        });
    };
};

} // namespace blazing
