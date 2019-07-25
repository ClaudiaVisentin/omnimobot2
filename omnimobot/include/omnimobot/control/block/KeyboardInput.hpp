#ifndef __CH_NTB_OMNIMOBOT_CONTROL_BLOCK_KEYBOARDINPUT_HPP
#define __CH_NTB_OMNIMOBOT_CONTROL_BLOCK_KEYBOARDINPUT_HPP

#include <eeros/control/Block1o.hpp>
#include <eeros/math/Matrix.hpp>
#include <thread>
#include <termios.h>

namespace omnimobot {

	class KeyboardInput: public eeros::control::Block1o<eeros::math::Vector<4>> {
	public:
		KeyboardInput();
		virtual ~KeyboardInput();
		virtual void run();
		
		eeros::control::Output<bool> esc;
		
	private:
		struct termios tio;
		char last;
		std::thread loop;
	};
};

#endif /* __CH_NTB_OMNIMOBOT_CONTROL_BLOCK_KEYBOARDINPUT_HPP */
