#ifndef __CH_NTB_OMNIMOBOT_SAFETYVELOCITYDESIRED_HPP
#define __CH_NTB_OMNIMOBOT_SAFETYVELOCITYDESIRED_HPP

#include "../../omnimobot/include/omnimobot/constants.hpp"

/**********************************************************
 * File:     SafetyVelocityDesired.hpp                              
 * Created:  2014  S.Landis                      
 * Changes:	 none
 * ------------------------------------------------------ 
 * Description:                                           
 * Calculated a safety velocity vector which no collision                                                                 
 **********************************************************/

class SafetyVelocityDesired {
	
public:
	SafetyVelocityDesired();
	~SafetyVelocityDesired();
	
	/** Calculate the safety velocity
	* @param collisionsData	Matrix(20,5) (collisions data) 0: collisionPoint_x, 1: collisionPoint_y, 2: angle, 3: distance, 4: index for the array sin cos tan array
	* @param sizeOfdataMatrix	desired velocity [m/s]
	*/
	virtual bool calcSafetyVeloDes(const CollisionDataMatrix& collisionsData, const DesiredVeloVector& desiredVeloIn  );

// 	virtual void setCalcObjectMove(bool isCalculate);
	virtual DesiredVeloVector& getSafetyVelo();
	
private:
	
	void calcSinCosTan();

	double cosAngle[omnimobot::angleSize];
	double sinAngle[omnimobot::angleSize];
	double tanAngle[omnimobot::angleSize];
	
	double desiredVelo_abs;
	double velocityRadial_abs;						// absolute value of the radial velocity of the collision points
	double gradientOfVelocity;
	double intersection_abs;
	
	int sitzeOfCollData;
	
	bool badVelocityRadial;
	bool isCalcObjectMove;
	
	DesiredVeloVector safetyDesVelo;

	IndexVector indexValuesAlreadyUsed;						// index liest of already considered collision points
	TmpVector2 velocityRadial;							// radial-velocity percentage of the collision points
	TmpVector2 velocityTangential;						// tangential-velocity percentage of the collision points
	TmpCalcVector ms;										// ms = gradient of the rectangular straight to radial-velocity
	TmpCalcVector qs;										// qs: straight: y = ms*x +qs
	TmpCalcMatrix intersection;								// tmpSaftyVelocity of intersection with rectangular straight to radial-velocity with the compute tmpSaftyVelocity
	TmpCalcMatrix velocityRadial_max;						// absolute value of the radial velocity of the collision points
	TmpCalcVector radial_max;
	IndexVector collisionPointIntersection;					// index liest	
	TmpCalcVector angleBetweenSafetyVeloAndCollPoint;		
	TmpCalcMatrix velocityRadialControl;
	
	BoolCalcVector ObstacleFront;			 // Obstacle is 1 = front (tan(w) = 0)
	BoolCalcVector ObstacleRectangular;		 // Obstacle is 1 = rectangular (tan(w) = nan)
// 	BoolCalcVector gradientDesVeloFront;			 // gradientDesVelo is 1 = front (tan(w) = 0)
// 	BoolCalcVector gradientDesVeloRectangular;		 // gradientDesVelo is 1 = rectangular (tan(w) = nan)

	
};



#endif /* __CH_NTB_OMNIMOBOT_SAFTYVELOCITYDESIRED_HPP */