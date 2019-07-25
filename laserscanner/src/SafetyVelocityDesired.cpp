#include <SafetyVelocityDesired.hpp>
#include "../../omnimobot/include/omnimobot/constants.hpp"
#include <unistd.h>

#include <iostream>

using namespace omnimobot;

SafetyVelocityDesired::SafetyVelocityDesired():

desiredVelo_abs(0.0),
sitzeOfCollData(0),
velocityRadial_abs(0.0),
gradientOfVelocity(0.0),
intersection_abs(0.0),
badVelocityRadial(false),
isCalcObjectMove(false)
{
	velocityRadial_max.zero();
	radial_max.zero();
	safetyDesVelo.zero();
	
	ObstacleFront.zero();
	ObstacleRectangular.zero();
// 	gradientDesVeloFront.zero();
// 	gradientDesVeloRectangular.zero();
	
	indexValuesAlreadyUsed.zero();
	velocityRadial.zero();
	velocityTangential.zero();
	ms.zero();
	qs.zero();
	intersection.zero();
	velocityRadial_max.zero();
	collisionPointIntersection.zero();
	angleBetweenSafetyVeloAndCollPoint.zero();
	
	calcSinCosTan();
}


SafetyVelocityDesired::~SafetyVelocityDesired()
{ 
	
}

DesiredVeloVector& SafetyVelocityDesired::getSafetyVelo()
{
	return safetyDesVelo;
}
 
// void SafetyVelocityDesired::setCalcObjectMove(bool isCalculate)
// {
// 	isCalcObjectMove = isCalculate;
// }


bool SafetyVelocityDesired::calcSafetyVeloDes(const CollisionDataMatrix& collisionsData, const DesiredVeloVector& desiredVeloIn )
{

	velocityRadial_max.zero();
	radial_max.zero();
	ms.zero();
	qs.zero();
	velocityRadial_abs = 0.0;
	intersection.zero();
	gradientOfVelocity = 0.0;
	desiredVelo_abs = 0.0;
	intersection_abs = 0.0;
	
	// If null then no restriction
	desiredVelo_abs = sqrt(desiredVeloIn(0)*desiredVeloIn(0) + desiredVeloIn(1)*desiredVeloIn(1));
	
	if (desiredVelo_abs <= 0.0005) {
		safetyDesVelo(0) = 0.0;
		safetyDesVelo(1) = 0.0;
		safetyDesVelo(2) = desiredVeloIn(2);
		return true;
	}
	
		
	// is a obstacle available (no obstacle)
	if ( collisionsData(0,3) == 0 ) {
		safetyDesVelo(0) = desiredVeloIn(0);
		safetyDesVelo(1) = desiredVeloIn(1);
		safetyDesVelo(2) = desiredVeloIn(2);
		return true;
	}
	
	// null values out
	for ( int i = 0; i < COLL_MATRIX_ROW; i++) {
		
		if (collisionsData(i,3) == 0) {
			sitzeOfCollData = i;
			break;
		}
		
		if ( i == COLL_MATRIX_ROW) {
			sitzeOfCollData = i+1;
		}
	}
	
	// if the deadzone distance achieved than saturation radial to 0
	for ( int i = 0; i < sitzeOfCollData; i++) {
		if ( collisionsData(i,3) < deadzone) {
			// reverse
			double phiColl = atan2(collisionsData(i,1),collisionsData(i,0));
			double phiDesVelo = atan2(desiredVeloIn(1),desiredVeloIn(0))+pi;
			double diff = phiColl - phiDesVelo; 
			double d = 0.0;
			// Check difference
			if(diff < 0){
				d += -diff;			
			}
			else {
				d += diff;
			}

			if (collisionsData(i,0) < 0 && desiredVeloIn(0) > 0 && d < 0.17 || collisionsData(i,0) > 0 && desiredVeloIn(0) < 0 && d < 0.17) { // 0.17 rad (range can be returned)
				safetyDesVelo(0) = desiredVeloIn(0) / 2.0;
				safetyDesVelo(1) = desiredVeloIn(1) / 2.0;
				safetyDesVelo(2) = desiredVeloIn(2);
			}
			else {
				safetyDesVelo(0) = 0.0;
				safetyDesVelo(1) = 0.0;
				safetyDesVelo(2) = desiredVeloIn(2);
			}
			return true;
		}
	}


	// radial Saturation
	double gradientColldata = 0.0;
	ObstacleFront.zero();
	ObstacleRectangular.zero();
	
	for ( int i = 0; i < sitzeOfCollData; i++) {
		
		if (collisionsData(i,3) < minPermissibleDistance) {
			radial_max(i) = 0.0;
		}
		else {
// 			radial_max(i) = maxVelo /(dangerzone - minPermissibleDistance) * collisionsData(i,3) - maxVelo * minPermissibleDistance/ (dangerzone-minPermissibleDistance);    // see doku   danger gleich wie scanzone
			radial_max(i) = sqrt(2*maxacc*(collisionsData(i,3) - minPermissibleDistance)); // see doku
		}

			
		// Vector of radial_max
		velocityRadial_max(i,0) = cos(collisionsData(i,2)) * radial_max(i);
		velocityRadial_max(i,1) = sin(collisionsData(i,2)) * radial_max(i); // collisionsData(i,4): index of the array sin cos tan array
		
		// rectangular straight to velocity_radial_max
// 		gradientColldata = tan(collisionsData(i,2));
		gradientColldata = collisionsData(i,1) / collisionsData(i,0);
		
		// if front
		if (gradientColldata == 0.0) {
			ObstacleFront(i) = true;
			qs(i) = velocityRadial_max(i,0);
		}
		// if rectangular
		else if (std::isnan(gradientColldata)) {
			ObstacleRectangular(i) = true;
			qs(i) = velocityRadial_max(i,1);
		}
		else {
			ms(i) = -1 / gradientColldata;     
			qs(i) = -velocityRadial_max(i,0) * ms(i) + velocityRadial_max(i,1); 
		}
	}
	
	
	
	// each collisions point is tested
	safetyDesVelo = desiredVeloIn;
	angleBetweenSafetyVeloAndCollPoint.zero();
	indexValuesAlreadyUsed.zero();
	indexValuesAlreadyUsed(0) = -1;
	velocityRadial.zero();
	velocityTangential.zero();
	

	double smallestAngle = 0.0;		            							//the smallest angle between velocity and collision point
	double angleSaftyVelo = atan2(safetyDesVelo(1),safetyDesVelo(0));	// angle of the tmpSaftyVelocity
	double projectionGain = 0.0; 												// projection of the actual tmpSaftyVelocity at the collisionPoint 
	bool first = true;
	bool check = false;
	bool noRelevantLimitation = false;
	int tmpIndex = 0;
	int sizeOfIndexValuesAlreadyUsed = 0;
	
	// compute the nearst obstacle due to the angle
	int sitzeOfAngleBetweenSaftyVeloAndCollPoint = 0;
	
	for (int u = 0; u < sitzeOfCollData; u++){
		angleBetweenSafetyVeloAndCollPoint(u,0) = fabs(collisionsData(u,2)-angleSaftyVelo);
		sitzeOfAngleBetweenSaftyVeloAndCollPoint = sitzeOfAngleBetweenSaftyVeloAndCollPoint + 1;
	}
	
	// compute the nearst collisions point, filtered collisions points are already indexValuesAlreadyUsed

	int tmp = 0;
	while(check == false) {
	
		for (int j = 0; j < sitzeOfCollData; j++){
		
			if(first){
				smallestAngle = angleBetweenSafetyVeloAndCollPoint(j,0);
				tmpIndex = j;
				first = false;
			}
			else {
				
				if (smallestAngle > angleBetweenSafetyVeloAndCollPoint(j,0)){
					
					if (sizeOfIndexValuesAlreadyUsed == 0){
						smallestAngle = angleBetweenSafetyVeloAndCollPoint(j,0);
						tmpIndex = j;
					}
					else {
					
						for (int n = 0; n < sizeOfIndexValuesAlreadyUsed; n++){
							
							if (indexValuesAlreadyUsed(n) == j){
								break;
							}
							
							if (n == sizeOfIndexValuesAlreadyUsed - 1){
								smallestAngle = angleBetweenSafetyVeloAndCollPoint(j,0);
								tmpIndex = j;								
							}
						}
					}
				}	
			}
		}
		
		// now we known: the index of the nearst collisions point based of the angle between velocity and collisions point
		
		// projection of the actual velocity at the collision point
		projectionGain =  (collisionsData(tmpIndex,0) * desiredVeloIn(0) + collisionsData(tmpIndex,1) * desiredVeloIn(1)) / (collisionsData(tmpIndex,3)*collisionsData(tmpIndex,3));  
		
		velocityRadial(0) = projectionGain * collisionsData(tmpIndex,0);			// [m/s]
		velocityRadial(1) = projectionGain * collisionsData(tmpIndex,1);
		
		// the tangetial part
		velocityTangential(0) = safetyDesVelo(0) - velocityRadial(0);
		velocityTangential(1) = safetyDesVelo(1) - velocityRadial(1);
		
		velocityRadial_abs = sqrt(velocityRadial(0)*velocityRadial(0) + velocityRadial(1)*velocityRadial(1) );
		
		badVelocityRadial = false;
		// test whether radial confinement is in the direction of the calculated speed
		if (velocityRadial(0) > 0.0 && collisionsData(tmpIndex,0) < 0.0  || velocityRadial(0) < 0.0 && collisionsData(tmpIndex,0) > 0.0) {
			badVelocityRadial = true;
		}
		
		if ( tmp >= sitzeOfCollData -1 &&  badVelocityRadial) {
            noRelevantLimitation = true;
            check = true;
		}
        else if (badVelocityRadial) {
           indexValuesAlreadyUsed(tmp) = tmpIndex; 
           sizeOfIndexValuesAlreadyUsed = sizeOfIndexValuesAlreadyUsed +1;
           tmp = tmp + 1; 
		}
        else {
            check = true;
		}
	}
	
	// Calculate safty velocity
// 	gradientDesVeloFront.zero();
// 	gradientDesVeloRectangular.zero();
// 	double gradientSafetyDesVelo = 0.0;
	
	if (radial_max(tmpIndex) < velocityRadial_abs) {
		velocityRadial(0) = velocityRadial_max(tmpIndex,0);
		velocityRadial(1) = velocityRadial_max(tmpIndex,1);
	}
	
	if(noRelevantLimitation) {
        return true;
	}
    else {
		safetyDesVelo(0) = velocityRadial(0) + velocityTangential(0);
		safetyDesVelo(1) = velocityRadial(1) + velocityTangential(1);
		
		// control of other collisions
		for ( int h = 0; h < sitzeOfCollData; h++) {
			
			if (tmpIndex == h) {
				continue;
			}
			
			desiredVelo_abs = sqrt(safetyDesVelo(0)*safetyDesVelo(0) + safetyDesVelo(1)*safetyDesVelo(1));
			
			gradientOfVelocity = safetyDesVelo(1) / safetyDesVelo(0);
			
			bool isnan = std::isnan(gradientOfVelocity);
			
			// velodes is front
			if(gradientOfVelocity == 0.0 && safetyDesVelo(0) > 0.0 && velocityRadial_max(h,0) > 0.0 && safetyDesVelo(0) > velocityRadial_max(h,0) ||
			   gradientOfVelocity == 0.0 && safetyDesVelo(0) < 0.0 && velocityRadial_max(h,0) < 0.0 && safetyDesVelo(0) < velocityRadial_max(h,0) ) {
				
				safetyDesVelo(0) = velocityRadial_max(h,0);
				std::cout << "velodes front" << std::endl;
				
			}
			// velodes is rectangular
			else if (isnan && safetyDesVelo(1) > 0.0 && velocityRadial_max(h,1) > 0.0 && safetyDesVelo(1) > velocityRadial_max(h,1) ||
				     isnan && safetyDesVelo(1) < 0.0 && velocityRadial_max(h,1) < 0.0 && safetyDesVelo(1) < velocityRadial_max(h,1)) {
				
				safetyDesVelo(1) = velocityRadial_max(h,1);
				std::cout << "velodes rectangular" << std::endl;
				
			}
			else {
				// has the safety velocity a intersection with another limitation
				
				// Obstacle  is rectangular
				if(ObstacleRectangular(h)){
					std::cout << "Hindernis rectangular" << std::endl;
					intersection(h,0) = qs(h) / gradientOfVelocity; 
					intersection(h,1) = qs(h);
				}
				// Obstacle is front
				else if(ObstacleFront(h)){
					std::cout << "Hindernis front" << std::endl;
					intersection(h,0) = qs(h) ; 
					intersection(h,1) = qs(h) * gradientOfVelocity;
					
				}
				else {
					intersection(h,0) = qs(h) / ( gradientOfVelocity - ms(h) ); 
					intersection(h,1) = qs(h) / ( gradientOfVelocity - ms(h) ) * gradientOfVelocity;
				}
											
				intersection_abs = sqrt(intersection(h,0)*intersection(h,0) + intersection(h,1)*intersection(h,1));
				
				// is the intersection nearer than the actual velocity
				if(desiredVelo_abs > intersection_abs && intersection(h,0) < 0.0 && intersection(h,0) > safetyDesVelo(0) && safetyDesVelo(0) < 0.0  || 
					desiredVelo_abs > intersection_abs && intersection(h,0) > 0.0 && intersection(h,0) < safetyDesVelo(0) && safetyDesVelo(0) > 0.0 ) {
					safetyDesVelo(0) = intersection(h,0);
					safetyDesVelo(1) = intersection(h,1);
				}
			}
		}
	}
	return true;
}



void SafetyVelocityDesired::calcSinCosTan()
{
	double tmpAngle = 0.0;
	
	for(int i = 0, step = 44; i < angleSize; i++, step++){		// step = 44 is the start step of the laserscanner
		
		if(step >= frontStep){
			tmpAngle = (step - frontStep) * angleResulution;
		}
		else{
			tmpAngle = -(frontStep - step )*angleResulution;
		}
		
		cosAngle[i] = cos(tmpAngle);
		sinAngle[i] = sin(tmpAngle);
		tanAngle[i] = tan(tmpAngle);
	}
}

