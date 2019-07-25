#ifndef __CH_NTB_OMNIMOBOT_CONTROL_CONTROLSYSTEMWITHLASER_HPP
#define __CH_NTB_OMNIMOBOT_CONTROL_CONTROLSYSTEMWITHLASER_HPP

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
#include <omnimobot/control/block/I.hpp>
#include <eeros/control/Gain.hpp>
#include <omnimobot/Counter.hpp>

#include <omnimobot/control/block/Jacobian.hpp>
#include <omnimobot/control/block/PendulumControl.hpp>
#include <eeros/control/Switch.hpp>
#include <omnimobot/control/block/HallSensorInput.hpp>
#include <omnimobot/control/block/TransLtoG.hpp>
#include <omnimobot/control/block/TransLtoG2.hpp>
#include <omnimobot/control/block/TransGtoSapostrophe.hpp>
#include <omnimobot/control/block/TransSapostropheToG.hpp>
#include <omnimobot/control/block/TransLtoSapostrophe.hpp>
#include <omnimobot/control/block/TransitionLaserData.hpp>
#include <omnimobot/control/block/SafetyVeloDesBlock.hpp>
#include <eeros/control/D.hpp>

#include <omnimobot/control/block/VoltageToPWM.hpp>
#include <omnimobot/control/block/RobotControl.hpp>
#include <types.hpp>
 
/**********************************************************
 * File:     ControlSystemWithLaser.hpp                              
 * Created:  2014  S.Landis                      
 * Changes:	 none
 * ------------------------------------------------------ 
 * Description:                                           
 * A control system for the OmniMoBot with pendulum control 
 * and collision detection                                              
 **********************************************************/

namespace omnimobot
{
	class ControlSystemWithLaser {
		
	public:
		ControlSystemWithLaser(double ts,TransitionLaserData& transLaserData);
 		virtual ~ControlSystemWithLaser();
		
		void start();
		void stop();
		bool isreadyToDrive();
		void initReadyToPendulum();
		bool isaxisHomed();
		void allAxisStopped();
		void setHomingToTrue();
		
		/** Using before drive	*/
		void initReadyToDrive();
		
		/** Using before homing	*/
		void initHoming();
		
		/** which steer should be homing
		* @param steer	steer1 = 3, steer2 = 4, steer3 = 5, */
		void setWheelToHoming(int steer);
		
		eeros::control::PeripheralInput<double> enc0;
		eeros::control::PeripheralInput<double> enc1;
		eeros::control::PeripheralInput<double> enc2;
		eeros::control::PeripheralInput<double> enc3;
		eeros::control::PeripheralInput<double> enc4;
		eeros::control::PeripheralInput<double> enc5;
		eeros::control::PeripheralOutput<double> pwmA0;
		eeros::control::PeripheralOutput<double> pwmA1;
		eeros::control::PeripheralOutput<double> pwmA2;
		eeros::control::PeripheralOutput<double> pwmA3;
		eeros::control::PeripheralOutput<double> pwmA4;
		eeros::control::PeripheralOutput<double> pwmA5;
		eeros::control::PeripheralOutput<double> pwmB0;
		eeros::control::PeripheralOutput<double> pwmB1;
		eeros::control::PeripheralOutput<double> pwmB2;
		eeros::control::PeripheralOutput<double> pwmB3;
		eeros::control::PeripheralOutput<double> pwmB4;
		eeros::control::PeripheralOutput<double> pwmB5;
		
		eeros::control::DeMux<6, double> deMuxPwmAOut;
		eeros::control::DeMux<6, double> deMuxPwmBOut;
		eeros::control::DeMux<4, double> demuxJoystick;
		eeros::control::DeMux<2, double> demuxVxyLocal;
		eeros::control::DeMux<2, double> deMuxPendulumOut;
		eeros::control::Mux<6, double> muxEncIn;
		eeros::control::Mux<3, double> muxVeloGlobalxyPhi;
		eeros::control::Mux<3, double> muxPendulumVeloDesired;
		eeros::control::Mux<2, double> muxVeloGlobal;
		eeros::control::Sum<2, Vector6> sumOffset;
		eeros::control::Constant<Vector6> offsetEnc;
		eeros::control::Gain<Vector6, double> gainDeltaQToQpoint;

		omnimobot::I<Vector6> integratorQpoint;
		omnimobot::I<eeros::math::Vector<3>> integratorOdoPointToOdo;
		omnimobot::I<eeros::math::Vector<2>> integratorVGlobalxyToPosGlobalxy;
		omnimobot::I<> integratorPhiPointglobalToPhiGL;
		omnimobot::RobotControl robotControlBlock;
		omnimobot::VoltageToPWM voltageToPwm;
		omnimobot::Jacobian jacobi;
		omnimobot::PendulumControl pendulumControl;
		omnimobot::HallSensorInput hallSensorInput;
		omnimobot::TransLtoG localToGlobalJacobi;
		omnimobot::TransLtoG2 localToGlobalHall;
		omnimobot::TransGtoSapostrophe transGtoS;
		omnimobot::TransSapostropheToG transStoG;
		omnimobot::TransLtoSapostrophe transLtoS;
		omnimobot::SafetyVeloDesBlock collisionAvoidance;
		
		omnimobot::XBoxInput joystick;
		
		eeros::control::Switch<2, eeros::math::Vector<3>> swPendulumOn; // Input 0 is not pendulum, Input 1 is Pendulum
		
		eeros::control::TimeDomain timedomain;
		
	private:
	
		void checkStartDrive();
		
		static eeros::math::Matrix<6,6,double> gear;					// Getriebe Übersetungen
		static eeros::math::Matrix<6,6,double> invGear;				
		static eeros::math::Matrix<6,6,double> inertia;				// Massenträgheiten
		static eeros::math::Matrix<6,1,double> torqueLim;								// Momenten Limit [Nm]
		static eeros::math::Matrix<6,1,double> speedLim;								// Geschwindigkeit Limit 1 [m/s] -> 20 rad/s (antrieb), 10.36 rad/s (lenkung)
		static eeros::math::Matrix<6,1,double> resetValue6;
		static eeros::math::Matrix<3,1,double> resetValue3;
		static eeros::math::Matrix<2,1,double> resetValue2;

// 		eeros::control::Constant<eeros::math::Vector<3>> safetyStopValue;
		bool ishomed;
		bool isReady;
		bool checkstartdrive;
		
		bool firstCounter;
		
		Counter counter;
		
	};
}








#endif /* __CH_NTB_OMNIMOBOT_CONTROL_CONTROLSYSTEMWITHLASER_HPP */