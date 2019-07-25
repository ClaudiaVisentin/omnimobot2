#ifndef __CH_NTB_OMNIMOBOT_CONSTANTS_HPP
#define __CH_NTB_OMNIMOBOT_CONSTANTS_HPP
#include <eeros/math/Matrix.hpp>
#include <cmath>

// Name of Vectors and matrices
#define MAX_MATRIX_ROW 682
#define COLL_MATRIX_ROW 20 // 20
typedef eeros::math::Matrix<MAX_MATRIX_ROW,1> DataVector;
typedef eeros::math::Vector<3> DesiredVeloVector; // velocity in x, y and rotation
typedef eeros::math::Vector<2> PosVector;
typedef eeros::math::Vector<2> TmpVector2;
typedef eeros::math::Matrix<COLL_MATRIX_ROW,1,int> IndexVector;
typedef eeros::math::Matrix<COLL_MATRIX_ROW,1> TmpCalcVector;
typedef eeros::math::Matrix<COLL_MATRIX_ROW,1,bool> BoolCalcVector;
typedef eeros::math::Matrix<MAX_MATRIX_ROW,1> TmpCalcMaxRowVector;
typedef eeros::math::Matrix<MAX_MATRIX_ROW,1,bool> BoolCalcMaxRowVector;
typedef eeros::math::Matrix<MAX_MATRIX_ROW,1,int> IndexMaxRowVector;
typedef eeros::math::Matrix<2,1,int> IndexSmallVector;
typedef eeros::math::Vector<3> VeloActualVector;
typedef eeros::math::Vector<3> PosDiffVector;
typedef eeros::math::Vector<2> PosTmpVector;

typedef eeros::math::Matrix<MAX_MATRIX_ROW,2> DataMatrix;
typedef eeros::math::Matrix<MAX_MATRIX_ROW,2,int> IndexMatrix;
typedef eeros::math::Matrix<COLL_MATRIX_ROW,5> CollisionDataMatrix;
typedef eeros::math::Matrix<COLL_MATRIX_ROW,2> TmpCalcMatrix;
typedef eeros::math::Matrix<2,2> RotMatrix;


namespace omnimobot
{
	static constexpr double pi = 3.14159265358979;
	static constexpr double S = 2.0;						// safty factor
	
	// Motor control parameter
	static constexpr double fControler = 400;					// frequency of control for the balance 400 
	static constexpr double dt = 1.0 / fControler;					// period of control
	static constexpr double dampingConst = 0.7;					// damping constante
	static constexpr double frequency0 = fControler/(4.0*pi*dampingConst*S);	
	static constexpr double omega0 = 2.0*pi*frequency0 / 2.0;	//  for the balance / 2.0  				
	static constexpr double km = 0.0302;					// Drehmomentkonstante
	static constexpr double R = 0.299;						// el.Widerstand
	static constexpr double kd = 2.0*dampingConst*omega0;			// see doku
	static constexpr double kp = (omega0 / (2.0*dampingConst));  // see doku
	static constexpr double bridgeVoltage = 24.0;			// bridge voltage (Speisung)

	// robot definition
	static constexpr double steerOffset = 0.0965049;			// see doku [m]
	static constexpr double radiusWheel = 0.05;						// see doku [m]
	static constexpr double omniLength = 0.392598183;			// [m]		
	static constexpr double deltax1 = omniLength;				// deltax = l*cos(alpha), alpha1=0, alpha2=(1/3)*2*pi, alpha3=(2/3)*2*pi, l=0.392598183
	static constexpr double deltax2 = -omniLength*0.5;	
	static constexpr double deltax3 = deltax2;	
	static constexpr double deltay1 = 0.0;						// deltay = l*sin(alpha), alpha1=0, alpha2=(1/3)*2*pi, alpha3=(2/3)*2*pi, l=0.392598183
	static constexpr double deltay2 = omniLength*0.866;
	static constexpr double deltay3 = -omniLength*0.866;
	static constexpr double minVelocity = 0.00000001;				// if the velocity minor than minVelocity is the actual velocity = 0.0
	static constexpr double permissibleDifference = 0.0001;
	static constexpr double homingspeed = 1.0;					// desired homingspeed [rad]
	
	
	// stick control parameter (Silvan)
	static constexpr double dampingTip = 0.7;
	static constexpr double frequencyTip = 7.7;
	static constexpr double frequencyTip0 = frequencyTip/(4.0*pi*dampingTip*S);
	static constexpr double omegaTip0 = 2.0*pi*frequencyTip0; // / 2.5;
	static constexpr double kpTip = omegaTip0*omegaTip0 / 5.0;    
	static constexpr double kdTip = 2.0*omegaTip0*dampingTip / 5.0;
	
	static constexpr double dampingAngle = 0.7;
	static constexpr double frequencyAngle = 42.0;
	static constexpr double frequencyAngle0 = frequencyAngle/(4.0*pi*dampingAngle*S);
	static constexpr double omegaAngle0 = 2.0*pi*frequencyAngle0; // / 2.0;
	static constexpr double kpPhi = omegaAngle0*omegaAngle0 / 4.0;//   / 3.0 ist zu wenig (zu nervös)
	static constexpr double kdPhi = 2.0*omegaAngle0*dampingAngle / 4.0;
	
	
 	// stick parameter
	static constexpr double posGravityPoint = 0.08;		//[m]
	static constexpr double inertiaStick = 0.00038;		//[kg*m²]
	static constexpr double mStick = 0.037;				//[kg]
	static constexpr double lengthOfStick = 0.324; // [m]
	static constexpr double gravity = 9.81; // [m/s²]
	static constexpr double rFactor = sqrt(inertiaStick/mStick); // see doku

	
 	// gear
	static constexpr double gearWheel = 18.0;
	static constexpr double gearSteer = 36.75;
 	
 	// inertia for testing on the backside
// 	static constexpr double inertiaWheel = 0.011962;  // [kg*m²]
// 	static constexpr double inertiaSteer = 0.029557;  // [kg*m²]
	// inertia
	static constexpr double inertiaWheel = 0.034279;  // [kg*m²]
	static constexpr double inertiaSteer = 0.0668063;  // [kg*m²]
	
	// saturation
	static constexpr double torqueSaturation = 0.53;  // [Nm] motorside
	static constexpr double speedLimWheel = 41.8;  // [rad/s] gearside
	static constexpr double speedLimSteer = 20.5;  // [rad/s] gearside
	
	
	static constexpr int steer1 = 3; // Wheel number 1
	static constexpr int steer2 = 4; // Wheel number 2
	static constexpr int steer3 = 5; // Wheel number 3
	
	// Offset steer
	static constexpr double offsetSteer1 = 1.776; // [rad]
	static constexpr double offsetSteer2 = 2.833; // [rad]
	static constexpr double offsetSteer3 = 5.6997; // [rad]
	
	
	// Sensorprint
	static constexpr double factor = 0.00175; // see doku
	static constexpr double bitFactor = 4095.0; // see doku
	static constexpr double fluxMin = 8300.0; // see doku 
	static constexpr double errorInx = 0.0; // see doku 
	static constexpr double errorIny = 0.024; // see doku 
	
	
	// Laserscanner
	static constexpr double dtLaser = 0.17; // [s] ev. noch anpassen

	static constexpr int startStep = 44; // start Step Laserscanner
	static constexpr int endStep = 725; // end Step Laserscanner
	static constexpr int frontStep = 384;
	static constexpr int totalSteps = 1024;
	
	// Collisions Detection
	static constexpr double transStoSapostrophe = 0.2075; // distance S to L
	static constexpr double scanzone = 1.5; // [m]
	static constexpr double dangerzone = 0.8; // [m]
	static constexpr double minPermissibleDistance = 0.55; // [m]
	static constexpr double deadzone = 0.45; // [m] mit 0.45 auch gut

	static constexpr double angleResulution = (2.0*pi)/totalSteps;		// to rad
	static constexpr int angleSize = 682;
	
	// SafetyVeloDesired
	static constexpr double maxVelo = 1.0; // [m/s]
	static constexpr double maxacc = 4.0; // [m/s²]
	
	// Measuring
	static constexpr int sizeDatalog = 15000; // 7000 = 7 sec with 500 Hz
	
}

#endif /* __CH_NTB_OMNIMOBOT_CONSTANTS_HPP */