#include <omnimobot/JoystickThread.hpp>

omnimobot::JoystickThread::JoystickThread(const char* device)
{
	joystick.open(device);
}


omnimobot::JoystickThread::~JoystickThread()
{
	joystick.setStop();
	joystick.close();
}

void omnimobot::JoystickThread::run()
{
    joystick.loop();
}


