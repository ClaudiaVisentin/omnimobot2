#ifndef __CH_NTB_OMNIMOBOT_JOYSTICKTHREAD_HPP
#define __CH_NTB_OMNIMOBOT_JOYSTICKTHREAD_HPP

#include <eeros/core/Thread.hpp>
#include <omnimobot/Joystick.hpp>
#include <fcntl.h>


namespace omnimobot {
	
	class JoystickThread : public eeros::Thread {
		
	public:
		JoystickThread(const char* device);
		~JoystickThread();
		virtual void run();

		remote::Joystick joystick;

	};
};
#endif /* __CH_NTB_OMNIMOBOT_JOYSTICKTHREAD_HPP */












