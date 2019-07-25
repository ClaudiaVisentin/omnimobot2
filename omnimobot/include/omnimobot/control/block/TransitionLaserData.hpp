#ifndef __CH_NTB_OMNIMOBOT_CONTROL_BLOCK_TransitionLaserData_HPP
#define __CH_NTB_OMNIMOBOT_CONTROL_BLOCK_TransitionLaserData_HPP

#include <types.hpp>
#include <eeros/math/Matrix.hpp>
// #include <eeros/control/TransitionBlock.hpp>
#include <eeros/control/Input.hpp>
#include <eeros/control/Output.hpp>
#include <constants.hpp>

#include <mutex>
#include <atomic>


namespace omnimobot
{

// 	class TransitionLaserData : public eeros::control::TransitionBlock
// 	{
// 	public:
// 		TransitionLaserData();
// 		virtual ~TransitionLaserData();
// 		
// 		virtual eeros::control::Input<VeloActualVector>& getInVeloActualSapostrophe() {
// 			return inVeloActualSap;
// 		}
// 		
// 		virtual eeros::control::Input<CollisionDataMatrix>& getInCollDataSapostrophe() {
// 			return inCollisionData;
// 		}
// 		
// 		virtual eeros::control::Output<CollisionDataMatrix>& getOutCollDataSapostrophe() {
// 			return outCollisionData;
// 		}
// 		
// 		virtual eeros::control::Output<bool>& getOutIsObjectMoveCalc() {
// 			return isObjectMoveCalc;
// 		}
// 			
// 		virtual void runA(); // timedomain slow
// 		virtual void runB(); // timedomain fast
// 
// 	private:
// 		
// 		virtual void calcObjectMove(VeloActualVector& veloActual, CollisionDataMatrix& collisionData, PosDiffVector& posDiffData, bool newData);
// 		
// 		eeros::control::Input<VeloActualVector> inVeloActualSap;
// 		eeros::control::Input<CollisionDataMatrix> inCollisionData;
// 		eeros::control::Output<CollisionDataMatrix> outCollisionData;
// 		eeros::control::Output<bool> isObjectMoveCalc;
// 		
// 		CollisionDataMatrix colldataA1;
// 		CollisionDataMatrix colldataA2;
// 		CollisionDataMatrix *colldata;
// 		PosDiffVector posDiffA1;
// 		PosDiffVector posDiffA2;
// 		PosDiffVector lastDiffPos;
// 		
// 		CollisionDataMatrix resetColl;
// 		VeloActualVector resetVelo;
// 		VeloActualVector inVeloActual;
// 		RotMatrix rot;
// 		RotMatrix rotSlow;
// 		PosTmpVector posTmp;
// 		eeros::control::Signal<CollisionDataMatrix> prev;
// 		
// 		double tin;
// 		double tprev;
// 		double dtTemp;
// 		double omegaTmp;
// 		std::atomic<bool> isrunA;
//  		volatile bool isCalcObjectMove;
// 
// 		std::mutex mutex;

// 	};
	
}
#endif /* __CH_NTB_OMNIMOBOT_CONTROL_BLOCK_TransitionLaserData_HPP */
