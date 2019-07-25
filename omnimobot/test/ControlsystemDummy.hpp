#ifndef __CH_NTB_OMNIMOBOT_CONTROLSYSTEMDUMMY_HPP
#define __CH_NTB_OMNIMOBOT_CONTROLSYSTEMDUMMY_HPP

#include <eeros/core/Runnable.hpp>
#include <eeros/math/Matrix.hpp>
#include <eeros/control/PeripheralInput.hpp>
#include <eeros/control/PeripheralOutput.hpp>
#include <eeros/math/Frame.hpp>
#include <omnimobot/control/block/XBoxInput.hpp>
#include <eeros/control/TimeDomain.hpp>
#include <eeros/control/Mux.hpp>
#include <eeros/control/DeMux.hpp>
#include <eeros/control/D.hpp>
#include <eeros/control/Saturation.hpp>


#include <omnimobot/control/block/VoltageToPWM.hpp>
#include <omnimobot/control/block/RobotControl.hpp>
#include <eeros/control/Switch.hpp>
#include <types.hpp>
 
// #include <eeros/control/Constant.hpp>
// #include <eeros/control/Sum.hpp>
// #include <eeros/control/Gain.hpp>

namespace omnimobot
{
	class ControlsystemDummy {
		
	public:
		ControlsystemDummy(double ts);
		
		void start();
		void stop();
		void controlOn();
		bool homeingFinished();
		bool axisHomed();
		void allAxisStopped();
		bool readyToHoming();
		
		
		// Methods for the Sequencer
		
		// Blocks
		
		eeros::control::Constant<eeros::math::Matrix<4,1,double>> joystickConst;
//		omnimobot::XBoxInput joystick;
		eeros::control::PeripheralInput<double> enc0;
		eeros::control::PeripheralInput<double> enc1;
		eeros::control::PeripheralInput<double> enc2;
		eeros::control::PeripheralInput<double> enc3;
		eeros::control::PeripheralInput<double> enc4;
		eeros::control::PeripheralInput<double> enc5;
// 		eeros::control::PeripheralOutput<double> pwmA0;
// 		eeros::control::PeripheralOutput<double> pwmA1;
// 		eeros::control::PeripheralOutput<double> pwmA2;
// 		eeros::control::PeripheralOutput<double> pwmA3;
// 		eeros::control::PeripheralOutput<double> pwmA4;
// 		eeros::control::PeripheralOutput<double> pwmA5;
// 		eeros::control::PeripheralOutput<double> pwmB0;
// 		eeros::control::PeripheralOutput<double> pwmB1;
// 		eeros::control::PeripheralOutput<double> pwmB2;
// 		eeros::control::PeripheralOutput<double> pwmB3;
// 		eeros::control::PeripheralOutput<double> pwmB4;
// 		eeros::control::PeripheralOutput<double> pwmB5;
		
// 		eeros::control::DeMux<6, double> deMuxPwmAOut;
// 		eeros::control::DeMux<6, double> deMuxPwmBOut;
		eeros::control::DeMux<4, double> demuxJoystick;
		eeros::control::Mux<6, double> muxEncPosIn;
		eeros::control::Mux<3, double> muxVeloLocalIn;
		eeros::control::Sum<2, Vector6> sumOffset;
		eeros::control::Constant<Vector6> offsetEnc;
		eeros::control::D<Vector6> diffEncPos;
//		eeros::control::Saturation<> pwmASaturation;
		
// 		omnimobot::RobotControl robotControlBlock;
// 		omnimobot::VoltageToPWM voltageToPwm;
		
		eeros::control::TimeDomain timedomain;
		
	private:
		// Motor- und Regelungsparameter
		static constexpr double km = 0.0302;					// Drehmomentkonstante
		static constexpr double R = 0.299;						// el.Widerstand
		static constexpr double kv = 250;  						// Geschwindigkeitsverstärkung normal: 500
		static constexpr double kp = 1.275020408163266e+02;		// Positionsverstärkung normal: 2.551020408163266e+02
		static constexpr double bridgeVoltage = 24.0;			// Brückenspannung (Speisung)

		static eeros::math::Matrix<6,6,double> gear;					// Getriebe Übersetungen
		static eeros::math::Matrix<6,6,double> invGear;				
		static eeros::math::Matrix<6,6,double> inertia;				// Massenträgheiten
		static eeros::math::Matrix<6,1,double> torqueLim;								// Momenten Limit [Nm]
		static eeros::math::Matrix<6,1,double> speedLim;								// Geschwindigkeit Limit 1 [m/s] -> 20 rad/s (antrieb), 10.36 rad/s (lenkung)
// 		eeros::math::Matrix<6,1,double> homingspeed;			// Geschwindigkeiten für das Homing
// 		eeros::math::Matrix<3,1,double> offset;					// Offset für die Enc
		
		bool homed;
		

		
	};
}








#endif /* __CH_NTB_OMNIMOBOT_CONTROLSYSTEMDUMMY_HPP */