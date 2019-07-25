#ifndef __CH_NTB_OMNIMOBOT_CONTROL_BLOCK_HOMINGBLOCK_HPP
#define __CH_NTB_OMNIMOBOT_CONTROL_BLOCK_HOMINGBLOCK_HPP

#include <eeros/control/Block.hpp>
#include <eeros/control/Input.hpp>
#include <eeros/control/Output.hpp>
#include <eeros/control/DeMux.hpp>
#include <eeros/control/Mux.hpp>
#include <eeros/control/Switch.hpp>
#include <eeros/control/Constant.hpp>
#include <eeros/math/Matrix.hpp>
#include <omnimobot/types.hpp>

namespace omnimobot {
	
	class HomingBlock : public eeros::control::Block {
		
	public:
		HomingBlock();
		virtual ~HomingBlock();
		
		virtual eeros::control::Input<Vector6>& getInTorque();
		
		virtual eeros::control::Output<Vector6>& getOutHomingTorque();
		
		virtual void run();
		
		eeros::control::Switch<2, double> swWeel1; // (1) konstante Null, (0) verwendet torqueInput. 
		eeros::control::Switch<2, double> swWeel2;
		eeros::control::Switch<2, double> swWeel3;
		eeros::control::Switch<2, double> swSteer1;
		eeros::control::Switch<2, double> swSteer2;
		eeros::control::Switch<2, double> swSteer3;
		
	private:
	
		eeros::control::Constant<> zeroTorque;
		eeros::control::DeMux<6, double> deMuxTorque;
		eeros::control::Mux<6, double> muxTorque;

	};
};
#endif /* __CH_NTB_OMNIMOBOT_CONTROL_BLOCK_HOMINGBLOCK_HPP */