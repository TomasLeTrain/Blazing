#include "pros/rtos.hpp"

class Motion {
	protected:
	virtual int getLoopDelayTime();
	virtual void execute();
	public:
	Motion(){}
	// waits until finishes
	void run(){
		while(true){
			this->execute();
			pros::delay(getLoopDelayTime());
		}
	};
	// runs async
	void async(){
		// spawn a task to run this in 
		pros::Task([this](){
			this->run();
		});
	};
};
