#include <omnimobot/control/block/KeyboardInput.hpp>
#include <eeros/core/System.hpp>

#include <iostream>
#include <stdio.h>
#include <unistd.h>
#include <termios.h>

using namespace omnimobot;


KeyboardInput::KeyboardInput() : loop([this]()
	{
		while (std::cin.good())
		{
			char c;
			std::cin.get(c);
			
			auto v = getOut().getSignal().getValue();
			
			if (c == 27) // ESC
			{
				esc.getSignal().setValue(true);
			}
			else if (c == '5')		// stop
			{
				v.zero();
			}
			else if (c == '8')		// forward
			{
				if (v[0] > -1.0)
				{
					v[0] -= 0.02;
				}
			}
			else if (c == '2')		// backward
			{
				if (v[0] < 1.0)
				{
					v[0] += 0.02;
				}
			}
			else if (c == '4')		// on the right
			{
				if (v[1] < 1.0)
				{
					v[1] += 0.02;
				}
			}
			else if (c == '6')		// on the left
			{
				if (v[1] > -1.0)
				{
					v[1] -= 0.02;// 0.02
				}
			}
			else if (c == '9')		// turn right
			{
				if (v[3] > -1.0)
				{
					v[3] -= 0.02;
				}
			}
			else if (c == '7')		// turn left
			{
				if (v[3] < 1.0)
				{
					v[3] += 0.02;
				}
			}
			
			for (int i = 0; i < 4; i++)
			{
				if (v[i] < -1.0)
					v[i] = -1.0;
				else if (v[i] > 1.0)
					v[i] = 1.0;
			}
			
			last = c;
			getOut().getSignal().setValue(v);
		}
	})
{
	tcgetattr(STDIN_FILENO, &tio);
	tio.c_lflag &=(~ICANON & ~ECHO);
	tcsetattr(STDIN_FILENO, TCSANOW, &tio);
	
	esc.getSignal().setValue(false);
	
	eeros::math::Vector<4> z;
	z.zero();
	getOut().getSignal().setValue(z);
	last = 0;
}

KeyboardInput::~KeyboardInput()
{
	tio.c_lflag |=(ICANON | ECHO);
	tcsetattr(STDIN_FILENO, TCSANOW, &tio);
}

void KeyboardInput::run()
{
	auto ts = eeros::System::getTimeNs();
	esc.getSignal().setTimestamp(ts);
	getOut().getSignal().setTimestamp(ts);
}
