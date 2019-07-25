#ifndef __CH_NTB_OMNIMOBOT_CONTROL_BLOCK_XBOXINPUT_HPP
#define __CH_NTB_OMNIMOBOT_CONTROL_BLOCK_XBOXINPUT_HPP

#include <string>
#include <thread>
#include <eeros/control/Output.hpp>
#include <eeros/control/Input.hpp>
#include <eeros/control/Block.hpp>
#include <eeros/core/System.hpp>
#include <omnimobot/Joystick.hpp>

#include <eeros/math/Matrix.hpp>



namespace omnimobot {

	class XBoxInput: public eeros::control::Block {
	public:
		XBoxInput(std::string dev);
		virtual ~XBoxInput();
		
		virtual void run();
		virtual bool isStop();
		virtual bool startMeasuring();
		virtual bool shoutDown();
		
		virtual eeros::control::Output<eeros::math::Vector<4>>& getOutVGdes() {
			return out;
		}
		
		virtual eeros::control::Input<eeros::math::Vector<3>>& getInVGactual() {
			return vIn;
		}
	
	
		remote::JoystickState state;
			
	private:

		eeros::control::Input<eeros::math::Vector<3>> vIn;
		eeros::control::Output<eeros::math::Vector<4>> out;
		
		eeros::math::Vector<2> in;
		double LTactual;
		double RTactual;
		eeros::math::Vector<4> vOut;
		eeros::math::Vector<4> vOutPrev;
		eeros::math::Vector<2> direction;
		
		bool stopDrive;
		
		double xyScale;
		double rotScale;
		double saturation;
		double saturationRot;
		
		int LButton;
		int RButton;

		double rot;
		
		remote::Joystick j;
		std::thread* t;
	};
};

#endif /* __CH_NTB_OMNIMOBOT_CONTROL_BLOCK_XBOXINPUT_HPP */
