#include <omnimobot/control/block/TransitionLaserData.hpp>

using namespace omnimobot;
using namespace eeros::math;


TransitionLaserData::TransitionLaserData():
omegaTmp(0.0),
tin(0.0),
tprev(0.0),
dtTemp(0.0),
isrunA(false)
{
	rot.zero();
	inVeloActual.zero();
	posTmp.zero();
	resetColl.zero();
	prev.setValue(resetColl);
	resetVelo.zero();
	rotSlow.zero();
	lastDiffPos.zero();
}


TransitionLaserData::~TransitionLaserData() { }

void TransitionLaserData::runA()
{
	CollisionDataMatrix *data = (colldata == &colldataA1) ? &colldataA2 : &colldataA1; 
	PosDiffVector *dataPos = (data == &colldataA1) ? &posDiffA1 : &posDiffA2; 
	
	auto &d = *data;
	auto &p = *dataPos;

	d = inCollisionData.getSignal().getValue();
		
	// transition S to S apostrophe and dt influence
	for (int i = 0; i < COLL_MATRIX_ROW; i++) { //COLL_MATRIX_ROW
		
		// if has an obstacle (distance of obstacle == 0)
		if (d(i,3) == 0.0) {
			break;
		}
		else {
			// tranlstion S to S apostrophe see doku// [m]
			d(i,0) = d(i,0) + transStoSapostrophe; 
			d(i,3) = sqrt(d(i,0)*d(i,0)+d(i,1)*d(i,1)); // distance = abs
		}
	}
	
	p.zero(); // runA always busy as the other runB
	
	// Transition
	{
		std::lock_guard<std::mutex> lock(mutex);
		
		colldata = data;
		isrunA = true;
	}
}



void TransitionLaserData::runB()
{
	bool move = true;
	
	if(isrunA) {
		std::lock_guard<std::mutex> lock(mutex);

		move = false;
		isrunA = false;
	}
		
	CollisionDataMatrix *data = colldata;
	PosDiffVector *dataPos = (data == &colldataA1) ? &posDiffA1 : &posDiffA2;
	
	
	auto &d = *data;
	auto &p = *dataPos;
	
	tprev = prev.getTimestamp() / 1000000000.0;
	tin = inVeloActualSap.getSignal().getTimestamp() / 1000000000.0;
	
	dtTemp = (tin - tprev);
	omegaTmp = inVeloActual(2);
	double PhiRot = omegaTmp*dtTemp;
	
	if (move) {	
		inVeloActual =  inVeloActualSap.getSignal().getValue();
		
		double cosPhiRot = cos(PhiRot);
		double sinPhiRot = sin(PhiRot);
		
		rot(0,0) = cosPhiRot;
		rot(1,0) = -sinPhiRot;
	
		rot(0,1) = sinPhiRot;
		rot(1,1) = cosPhiRot;
		
		calcObjectMove(inVeloActual, d, p, false);
	}
	else {
		double cosPhiRot = cos(p(2));
		double sinPhiRot = sin(p(2));
		
		rot(0,0) = cosPhiRot;
		rot(1,0) = -sinPhiRot;
	
		rot(0,1) = sinPhiRot;
		rot(1,1) = cosPhiRot;
		calcObjectMove(inVeloActual, d, lastDiffPos, true);
	}
	
	
	p(0) = p(0) + inVeloActual(0)*dtTemp;
	p(1) = p(1) + inVeloActual(1)*dtTemp;
	p(2) = p(2) + PhiRot;
	
	lastDiffPos = p;
	
	isObjectMoveCalc.getSignal().setValue(move);
	isObjectMoveCalc.getSignal().setTimestamp(inVeloActualSap.getSignal().getTimestamp());
	outCollisionData.getSignal().setValue(d);
	outCollisionData.getSignal().setTimestamp(inVeloActualSap.getSignal().getTimestamp());
	prev = outCollisionData.getSignal();
}



void TransitionLaserData::calcObjectMove(VeloActualVector& veloActual, CollisionDataMatrix& collisionData, PosDiffVector& posDiffData, bool newData)
{	
	for (int i = 0; i < COLL_MATRIX_ROW; i++) { 
		
		// if has an obstacle (distance of the first obstacle == 0)
		if (collisionData(i,3) == 0.0) {
			return;
		}
		else {
			
			if (newData) {
				posTmp(0) = collisionData(i,0) - posDiffData(0);
				posTmp(1) = collisionData(i,1) - posDiffData(1);

			}
			else {
				posTmp(0) = collisionData(i,0) - veloActual(0)*dtTemp;
				posTmp(1) = collisionData(i,1) - veloActual(1)*dtTemp;
				
			}
			
			posTmp = rot * posTmp;
			collisionData(i,0) = posTmp(0);
			collisionData(i,1) = posTmp(1);
				
			collisionData(i,2) = atan2(collisionData(i,1),collisionData(i,0));
			collisionData(i,3) = sqrt(collisionData(i,1)*collisionData(i,1)+collisionData(i,0)*collisionData(i,0));
		}
	}
}



