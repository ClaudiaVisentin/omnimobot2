#ifndef __CH_NTB_OMNIMOBOT_COLLISIONDETECTION_HPP
#define __CH_NTB_OMNIMOBOT_COLLISIONDETECTION_HPP

#include "../../omnimobot/include/omnimobot/constants.hpp"

/**********************************************************
 * File:     CollisionDetection.hpp                              
 * Created:  2014  S.Landis                      
 * Changes:	 none
 * ------------------------------------------------------ 
 * Description:                                           
 * Calculate the nearst collisions data                                                                     
 **********************************************************/

class CollisionDetection {

public:
	CollisionDetection();
	~CollisionDetection();
	
	/** Calculate the nearst collisions data
	* @param distanceAndAngle	distance [m] and Angle [rad]
	* @param sizeOfdataMatrix	
	*/
	virtual bool calcCollisionsData(const DataMatrix& distanceAndAngle, int sizeOfdataMatrix);	
	
	virtual CollisionDataMatrix& getCollisionsData();
	
	virtual int& getSizeCollisionsData();
	
private:
	void calcSinCosTan();

	double cosAngle[omnimobot::angleSize];
	double sinAngle[omnimobot::angleSize];
	double tanAngle[omnimobot::angleSize];
	
	int sizeCollisionsData;	
	
	IndexMatrix criticalPoints;				// point in dangerzone (index register), Colums: 0 = index of dataMatrix, 1 = index  of the array cos,sin,tan
	IndexMatrix collisionPointsTmp;				// temp index register of de collision points, it's possible that it not e collision point but it is the nearst of all critical Points, Colums: 0 = index of dataMatrix, 1 = index of cos,sin,tan
	IndexMatrix collisionPointsdefinitely;			// temp index register of de collision points definitely
	
	
	TmpCalcMaxRowVector ms;					 // ms = gradient of the rectangular straight to critical point
	TmpCalcMaxRowVector qs;					 // qs: straight: y = ms*x +qs (tangent)
	
	IndexSmallVector nearestCriticalPoint;			 // 0 = index of dataMatrix, 1 = index of the array cos,sin,tan

	IndexMaxRowVector badCollisionPoints; 			 // collision points which in ather collision points included (index list)

	DataMatrix detectedPoints;				 // x-, y-values of the detected points
	DataVector angle;
	DataVector distance;					 // distance in coordinate system KS {S} (see doku)

	CollisionDataMatrix collData;		                 //0: collisionPoint_x, 1: collisionPoint_y, 2: angle, 3: distance, 4: index of the array sin cos tan array
	
	BoolCalcMaxRowVector gradientTangentFrontstep;		 // gradientTangent is 1 = frontstep (tan(w) = 0)
	BoolCalcMaxRowVector gradientDistanzFrontstep;		 // actual collpoint is 1 = frontstep (tan(w) = 0)
	BoolCalcMaxRowVector gradientTangentRectangular;	 // gradientTangent is 1 = rectangular (tan(w) = nan)
	BoolCalcMaxRowVector gradientDistanzRectangular;	 // actual collpoint is 1 = rectangular (tan(w) = nan)
	
};



#endif /* __CH_NTB_OMNIMOBOT_COLLISIONDETECTION_HPP */