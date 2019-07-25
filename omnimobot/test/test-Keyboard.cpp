#include <iostream>
#include <unistd.h>

#include <omnimobot/control/block/KeyboardInput.hpp>

int main(int argc, char *argv[])
{
	omnimobot::KeyboardInput input;
	
	while (!input.esc.getSignal().getValue())
	{
		input.run();
		usleep(1000);
		
		static int counter = 0;
		if (counter++ >= 200)
		{
			counter = 0;
			
			auto v = input.getOut().getSignal().getValue();
			std::cout << v[0] << "   " << v[1] << "   " << v[3] << std::endl;
		}
	}
	
	return 0;
}