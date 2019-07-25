#include <CollisionDetection.hpp>

#include <iostream>
#include <cmath>

using namespace omnimobot;

CollisionDetection::CollisionDetection():
sizeCollisionsData(1)
{
	criticalPoints.zero();
	collisionPointsTmp.zero();
	collisionPointsdefinitely.zero();
	
	ms.zero();
	qs.zero();
	
	gradientTangentFrontstep.zero();
	gradientDistanzFrontstep.zero();
	gradientTangentRectangular.zero();
	gradientDistanzRectangular.zero();
	
	nearestCriticalPoint.zero();
	badCollisionPoints.zero();
	
	detectedPoints.zero();
	angle.zero();
	distance.zero();
	
	collData.zero();
	
	calcSinCosTan();
}

CollisionDetection::~CollisionDetection()
{

}


bool CollisionDetection::calcCollisionsData( const DataMatrix& distanceAndAngle, int sizeOfdataMatrix)
{

	detectedPoints.zero();
	angle.zero();
	distance.zero();
	
	criticalPoints.zero();
	
	const int startIndexCosSinTan = startStep - 44;		// 44 is the start step of the laserscanner 
	
	
	// scandata in x,y-value and controll if point in scanzone 
	for (int i_in = 0, i_out = 0, u = startIndexCosSinTan; i_out < sizeOfdataMatrix; i_out++,u++){
		
		angle(i_out) = distanceAndAngle(i_out,1);
		distance(i_out) = distanceAndAngle(i_out,0);
		detectedPoints(i_out,0) = cosAngle[u] * distance(i_out); // [m] x
		detectedPoints(i_out,1) = sinAngle[u] * distance(i_out); // [m] y
		
		if (distance(i_out) <= scanzone){
			criticalPoints(i_in,0) = i_out;
			criticalPoints(i_in,1) = u;
			i_in++;
		}
	}

	// no critical points (indexmatrix: == 0 means that the index null comes in third agency)
	if (criticalPoints(2,0) == 0){		
		collData.zero();
		sizeCollisionsData = 0;
		return true;
	}
	
	// distance of the nearest critical point
	bool first = true;
	nearestCriticalPoint.zero();							// index 
	int sizeOfCriticalPoints = 1;
	
	for (int i = 0; i < sizeOfdataMatrix; i++){
		
		if (first){
			nearestCriticalPoint(0) = criticalPoints(i,0);
			nearestCriticalPoint(1) = criticalPoints(i,1);
			first = false;
		}
		else {
			
			if (criticalPoints(i,0) == 0){
				sizeOfCriticalPoints = i;
				break;
			}
			
			if (distance(nearestCriticalPoint(0)) > distance(criticalPoints(i,0))){
				nearestCriticalPoint(0) = criticalPoints(i,0);
				nearestCriticalPoint(1) = criticalPoints(i,1);
			}
		}
		
		if (i == sizeOfdataMatrix -1){
			sizeOfCriticalPoints = i + 1;
		}
	}
	

	
	// which critical points is a collision point
	collisionPointsTmp.zero();
	collisionPointsTmp(0,0) = nearestCriticalPoint(0);				
	ms.zero();
	qs.zero();
	gradientTangentFrontstep.zero();
	gradientDistanzFrontstep.zero();
	gradientTangentRectangular.zero();
	gradientDistanzRectangular.zero();
	
	double gradientTangente = 0.0;
	double gradientCriticalpoint = 0.0;											// gradient of the straight "critical point"
	double intersection_x = 0.0;												// position x of intersection with rectangular straight with another critical point
// 	double angle_tmp = 0.0;
// 	bool msNaN = false;
// 	bool graradientNaN = false;
	
	int sizeOfTmpCollisionPoints = 1;
	
	bool ifNewCollisionpoint = false;
	first = true;
	
	// test if it a collision critical point
	for (int i = 0, v = 0; i < sizeOfCriticalPoints; i++){						// i: index of the critical points, v: index of the tmp collision points (the first index is nearestCriticalPoint)
		
// 		angle_tmp = angle(criticalPoints(i,0));
		
		if (criticalPoints(i,0) == nearestCriticalPoint(0)){
			continue;
		}
		else {
			gradientCriticalpoint = tanAngle[criticalPoints(i,1)];
			// if frontstep
			if(gradientCriticalpoint == 0.0) {
				gradientDistanzFrontstep(i) = true;
			}
			// if rectangular
			else if (std::isnan(gradientCriticalpoint)) {
				gradientDistanzRectangular(i) = true;
			}

			// test if there existing points include the new
			for (int k = 0; k < sizeOfTmpCollisionPoints; k++){					// k: index of collision critical point
				
				if (first){														// if it the first collision critical point
					gradientTangente = tanAngle[nearestCriticalPoint(1)];
					// if frontstep (frontstep is the nearst collpoint -> qs is the x-Value) 
					if(gradientTangente == 0.0) {
						gradientTangentFrontstep(k) = true;
						qs(k) = detectedPoints(nearestCriticalPoint(0),0);
					}
					// if rectangular (the nearst collpoint is rectangular -> qs is the y-Value)
					else if (std::isnan(gradientTangente)) {
						gradientTangentRectangular(k) = true;
						qs(k) = detectedPoints(nearestCriticalPoint(0),1);
					}
					else {
						ms(k) = -1 / tanAngle[nearestCriticalPoint(1)];
						qs(k) = -detectedPoints(nearestCriticalPoint(0),0) * ms(k) + detectedPoints(nearestCriticalPoint(0),1);
					}
				
					first = false;
				}
				
				// distance or nearst is rectangular
				if(gradientDistanzRectangular(i) || gradientTangentRectangular(k)){
					if (detectedPoints(criticalPoints(i,0),1) > 0.0 && qs(k) > 0.0 && detectedPoints(criticalPoints(i,0),1) > qs(k) || 
						detectedPoints(criticalPoints(i,0),1) < 0.0 && qs(k) < 0.0 && detectedPoints(criticalPoints(i,0),1) < qs(k)){
						break;
					}
					else {
						// no collision piont include critical point -> new collision piont
						if (k == sizeOfTmpCollisionPoints - 1){
							k = k + 1;
// 							ms(k) = -1 / gradientCriticalpoint; // ms is nan
							qs(k) = detectedPoints(criticalPoints(i,0),1); // qs = y-value
							
							ifNewCollisionpoint = true;
						}
					}
				}
				// distance or nearst is frontstep
				else if(gradientDistanzFrontstep(i) || gradientTangentFrontstep(k)){
					if (detectedPoints(criticalPoints(i,0),0) > 0.0 && qs(k) > 0.0 && detectedPoints(criticalPoints(i,0),0) > qs(k) || 
						detectedPoints(criticalPoints(i,0),0) < 0.0 && qs(k) < 0.0 && detectedPoints(criticalPoints(i,0),0) < qs(k)){
						break;
					}
					else {
						// no collision piont include critical point -> new collision piont
						if (k == sizeOfTmpCollisionPoints - 1){
							k = k + 1;
// 							ms(k) = -1 / gradientCriticalpoint; // ms is nan
							qs(k) = detectedPoints(criticalPoints(i,0),0); // qs = y-value
							
							ifNewCollisionpoint = true;
						}
					}
				}
				else {
					intersection_x = qs(k) / (gradientCriticalpoint - ms(k) );
				
					// if the collision critical point include the new critical point
					if (intersection_x > 0.0 && intersection_x < detectedPoints(criticalPoints(i,0),0) && detectedPoints(criticalPoints(i,0),0) > 0.0 || 
						intersection_x < 0.0 && intersection_x > detectedPoints(criticalPoints(i,0),0) && detectedPoints(criticalPoints(i,0),0) < 0.0){
						break;
					}
					else {
						// no collision piont include critical point -> new collision piont
						if (k == sizeOfTmpCollisionPoints - 1){
							k = k + 1;
							ms(k) = -1 / gradientCriticalpoint;
							qs(k) = -detectedPoints(criticalPoints(i,0),0) * ms(k) + detectedPoints(criticalPoints(i,0),1); 
							
							ifNewCollisionpoint = true;
						}
					}
				}
			}
			
			// add new collision point
			if (ifNewCollisionpoint){
				v = v + 1;
				collisionPointsTmp(v,0) = criticalPoints(i,0);
				collisionPointsTmp(v,1) = criticalPoints(i,1);
				sizeOfTmpCollisionPoints = sizeOfTmpCollisionPoints + 1;
				ifNewCollisionpoint = false;
			}
		}
	}
	
	
	
	// bad collision critiacalpoints (collision points test each other)
	
	gradientDistanzFrontstep.zero();
	gradientDistanzRectangular.zero();
	
	double mOfTested = 0.0;											// gradient of de tested collisionPointsTmp
	
	double intersectionXOfTester = 0.0;								// intersection of the rectangular straight collision point (index i) to the collision point to be tested (index w)
	
	badCollisionPoints.zero();
	int sizeOfbadCollisionPoints = 0;								// 0 gesetzt
	
	bool ifNewBadCollisionpoint = false;
	bool ifBadPoint = false;
	
	for (int i = sizeOfTmpCollisionPoints, p = 0; i > 0; i--){		// auf den 0. index von dem collisionPointsTmp wurden schon alle getestet, p: index of the badCollisionPoints ( - 1 weg size)

		for (int v = 0; v < sizeOfbadCollisionPoints; v++){				// if collisionPointsTmp(v) == badpoint
			
			if (badCollisionPoints(v) == collisionPointsTmp(i,0)){
				ifBadPoint = true;
				break;
			}
		}
		
		if (ifBadPoint){
			ifBadPoint = false;
			continue;
		}

		for (int w = 0; w < sizeOfTmpCollisionPoints -1; w++){		// w: index von den collisionPointsTmp, die getestet werden (- 2 weg size)

			if (i == w){
				continue;
			}
			
			mOfTested = tanAngle[collisionPointsTmp(w,1)];
			// if frontstep
			if(mOfTested == 0.0) {
				gradientDistanzFrontstep(w) = true;
			}
			// if rectangular
			else if (std::isnan(mOfTested)) {
				gradientDistanzRectangular(w) = true;
			}
			
			// distance or nearst is rectangular
			if (gradientDistanzRectangular(w) || gradientTangentRectangular(i)){
				
				if (detectedPoints(collisionPointsTmp(w,0),1) > 0.0 && qs(i) > 0.0 && detectedPoints(collisionPointsTmp(w,0),1) > qs(i) || 
					detectedPoints(collisionPointsTmp(w,0),1) < 0.0 && qs(i) < 0.0 && detectedPoints(collisionPointsTmp(w,0),1) < qs(i)){
					
					for (int u = 0; u < sizeOfbadCollisionPoints + 1; u++){		
						
						if (badCollisionPoints(u) == collisionPointsTmp(w,0)){
							break;
						}
						
						if (u == sizeOfbadCollisionPoints) {				
							badCollisionPoints(p) = collisionPointsTmp(w,0);
							p = p + 1;
							ifNewBadCollisionpoint = true;
						}
					}
				}
			}
			// distance or nearst is frontstep
			else if (gradientDistanzFrontstep(w) || gradientTangentFrontstep(i)){
				
				if (detectedPoints(collisionPointsTmp(w,0),0) > 0.0 && qs(i) > 0.0 && detectedPoints(collisionPointsTmp(w,0),0) > qs(i) || 
					detectedPoints(collisionPointsTmp(w,0),0) < 0.0 && qs(i) < 0.0 && detectedPoints(collisionPointsTmp(w,0),0) < qs(i)){

					for (int u = 0; u < sizeOfbadCollisionPoints + 1; u++){		
						
						if (badCollisionPoints(u) == collisionPointsTmp(w,0)){
							break;
						}
						
						if (u == sizeOfbadCollisionPoints) {				
							badCollisionPoints(p) = collisionPointsTmp(w,0);
							p = p + 1;
							ifNewBadCollisionpoint = true;
						}
					}
				}
			}
			else {
				intersectionXOfTester = qs(i) / (mOfTested - ms(i));

				// if tmpCollisionPoint i better than tmpCollisionPoint w -> new badCollisionPoint (w)
				if (intersectionXOfTester > 0.0 && intersectionXOfTester < detectedPoints(collisionPointsTmp(w,0),0) && detectedPoints(collisionPointsTmp(w,0),0) > 0.0 || 
					intersectionXOfTester < 0.0 && intersectionXOfTester > detectedPoints(collisionPointsTmp(w,0),0) && detectedPoints(collisionPointsTmp(w,0),0) < 0.0 ) {
					
					for (int u = 0; u < sizeOfbadCollisionPoints + 1; u++){		
						
						if (badCollisionPoints(u) == collisionPointsTmp(w,0)){
							break;
						}
						
						if (u == sizeOfbadCollisionPoints) {				
							badCollisionPoints(p) = collisionPointsTmp(w,0);
							p = p + 1;
							ifNewBadCollisionpoint = true;
						}
					}
				}
			}
						
			// add new badCollisionPoint
			if (ifNewBadCollisionpoint){
				sizeOfbadCollisionPoints = sizeOfbadCollisionPoints + 1;
				ifNewBadCollisionpoint = false;
			}
		}
	}
	

	// collisionpoints without badcollisionpoints -> collisionpoints definitely

	collisionPointsdefinitely.zero();
	collData.zero();
	int sizeOfCollisiondefinitely = 0;
	bool ifNewCollisionPointsdefinitely = false;
	
	if (sizeOfbadCollisionPoints > 1){ 
		
		for (int i = 0; i < sizeOfTmpCollisionPoints; i++){
			
			for (int w = 0; w < sizeOfbadCollisionPoints; w++){
				
				if (collisionPointsTmp(i,0) != badCollisionPoints(w)){
					
					if (w == sizeOfbadCollisionPoints - 1){ // all collisionPointsTmp are checked
						
						ifNewCollisionPointsdefinitely = true;
				
						break;
					}
					continue;
					
				}
				else {
					break;
				}
			}
			
			// add new ifNewCollisionPointsdefinitely
			if (ifNewCollisionPointsdefinitely){
				collisionPointsdefinitely(sizeOfCollisiondefinitely,0) = collisionPointsTmp(i,0);
				collisionPointsdefinitely(sizeOfCollisiondefinitely,1) = collisionPointsTmp(i,1);
				sizeOfCollisiondefinitely = sizeOfCollisiondefinitely + 1;
				ifNewCollisionPointsdefinitely = false;
			}
		}
		
		for (int i = 0; i < sizeOfCollisiondefinitely; i++){
			
			if(i == COLL_MATRIX_ROW){
				break;
			}
			
			collData(i,0) = detectedPoints(collisionPointsdefinitely(i,0),0);
			collData(i,1) = detectedPoints(collisionPointsdefinitely(i,0),1);
			collData(i,2) = angle(collisionPointsdefinitely(i,0));
			collData(i,3) = distance(collisionPointsdefinitely(i,0));
			collData(i,4) = collisionPointsdefinitely(i,1);
		}
		
		if(sizeOfCollisiondefinitely <= COLL_MATRIX_ROW){
			sizeCollisionsData = sizeOfCollisiondefinitely;		
		}
		else{
			sizeCollisionsData = COLL_MATRIX_ROW;
		}
	}
	else {
		
		for (int i = 0; i < sizeOfTmpCollisionPoints; i++){
			
			if(i == COLL_MATRIX_ROW){
				break;
			}
			
			collData(i,0) = detectedPoints(collisionPointsTmp(i,0),0);
			collData(i,1) = detectedPoints(collisionPointsTmp(i,0),1);
			collData(i,2) = angle(collisionPointsTmp(i,0));
			collData(i,3) = distance(collisionPointsTmp(i,0));
			collData(i,4) = collisionPointsTmp(i,1);
		}
		
		if(sizeOfTmpCollisionPoints <= COLL_MATRIX_ROW){
			sizeCollisionsData = sizeOfTmpCollisionPoints;		
		}
		else{
			sizeCollisionsData = COLL_MATRIX_ROW;
		}
	}
	
	return true;
}



void CollisionDetection::calcSinCosTan()
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




int& CollisionDetection::getSizeCollisionsData()
{
	return sizeCollisionsData;
}



CollisionDataMatrix& CollisionDetection::getCollisionsData()
{
	return collData;
}


