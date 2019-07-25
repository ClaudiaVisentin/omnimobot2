// #include <omnimobot/JoystickThread.hpp>
#include <omnimobot/control/block/XBoxInput.hpp>
#include <iostream>
#include <unistd.h>

#define JOYSTICK_DEVICE "/dev/input/js0"


using namespace omnimobot;
using namespace eeros;

class Worker {
	public: Worker():
	xboxIn(JOYSTICK_DEVICE)
	{
		
	}
	
	void run() 
	{
		xboxIn.run();
		
// 		std::cout << "Out0 = "  << xboxIn.getOut().getSignal().getValue()(0)<<std::endl;
		
	}

	
	omnimobot::XBoxInput xboxIn;
		
};


static volatile bool running = true;
void sig_handler(int signum)
{
    running = false;
}


int main(int argc, char *argv[])
{
	std::cout << "Test start "  << std::endl;
	
	Worker test;
	
// 	while(running) {
// 		test.run();
// 	}
	
	
// 	JoystickThread thread(JOYSTICK_DEVICE);
	
	
// 	thread.joystick.on_button([] (int button, bool value) {
// 		std::cout << "button " << button << " = " << value << std::endl;
// 	});
	
	
// 	for (int i = 0; i < 20; i++) 
// 	{
// 		std::cout << "axis0 = " << thread.joystick.current.axis[0] << std::endl;
// 		usleep(500000);
// 	}
	int ctr = 0;
	
	for (int i = 0; i < 25000; i++) 
	{
		test.run();
		
		if(ctr == 250){
		
			std::cout << "Out0 = "  << test.xboxIn.getOutVGdes().getSignal().getValue()(0);
			std::cout << "    Out1 = "  << test.xboxIn.getOutVGdes().getSignal().getValue()(1);
// // 			std::cout << "    Out2 = "  << test.xboxIn.getOut().getSignal().getValue()(2);
			std::cout << "    Out3 = "  << test.xboxIn.getOutVGdes().getSignal().getValue()(3);
// // 			std::cout << "    Out3 = "  << test.xboxIn.in(2);
// // 			std::cout << "    RTactual = "  << test.xboxIn.RTactual;
// // 			std::cout << "    LTactual = "  << test.xboxIn.LTactual;
			std::cout << "    stop = "  << test.xboxIn.isStop()<< "    "  ;
			
// 			std::cout << "Out0 = "  << test.xboxIn.in(0);
// 			std::cout << "    Out1 = "  << test.xboxIn.in(1);
// // 			std::cout << "    Out2 = "  << test.xboxIn.getOut().getSignal().getValue()(2);
// 			std::cout << "    Out3 = "  << test.xboxIn.in(2);
// 			std::cout << "    stop = "  << test.xboxIn.isStop()<<std::endl;
			
			// axis
// 			for (int j = 0; j < 8; j++)
// 			{
// 				std::cout << j << ": ";
// 				std::cout << test.xboxIn.state.axis[j] << '\t';
// 			}
// 			std::cout  << std::endl;
			
			// button
			for (int j = 0; j < 16; j++)
			{
				std::cout << j << ": ";
				std::cout << test.xboxIn.state.button_state[j] << '\t';
			}
			std::cout  << std::endl;
			
			ctr = 0;
		}
		usleep(2000);
		
		ctr++;
	}
	
	
	std::cout << "Test finisch "  << std::endl;
	
	return 0;
}