#ifndef __CH_NTB_OMNIMOBOT_CONTROL_BLOCK_SAFETYVELODESBLOCK_HPP
#define __CH_NTB_OMNIMOBOT_CONTROL_BLOCK_SAFETYVELODESBLOCK_HPP

#include <omnimobot/constants.hpp>
#include <eeros/math/Matrix.hpp>
#include <eeros/control/Block.hpp>
#include <eeros/control/Output.hpp>
#include <eeros/control/Input.hpp>
#include "../../../../laserscanner/include/SafetyVelocityDesired.hpp"


namespace omnimobot
{

	class SafetyVeloDesBlock : public eeros::control::Block
	{
	public:
		SafetyVeloDesBlock();
		virtual ~SafetyVeloDesBlock();
			
		virtual eeros::control::Output<DesiredVeloVector>& getOutSafetyVeloSapostrophe() {
			return outSafetyVelo;
		}
		virtual eeros::control::Input<DesiredVeloVector>& getInVeloDesSapostrophe() {
			return inVeloDesired;
		}
		virtual eeros::control::Input<CollisionDataMatrix>& getInColldataSapostrophe() {
			return inColldata;
		}
		virtual eeros::control::Input<bool>& getInisMoveCalc() {
			return inIsMoveCalc;
		}

		virtual void run();


	private:
	
		eeros::control::Output<DesiredVeloVector> outSafetyVelo;
		eeros::control::Input<DesiredVeloVector> inVeloDesired;
		eeros::control::Input<CollisionDataMatrix> inColldata;
		eeros::control::Input<bool> inIsMoveCalc;
		
		CollisionDataMatrix colldata;
		DesiredVeloVector veloDes;
		DesiredVeloVector safetyVelo;
		SafetyVelocityDesired safetyVeloDefine;

	};
	
}
#endif /* __CH_NTB_OMNIMOBOT_CONTROL_BLOCK_SAFETYVELODESBLOCK_HPP */
