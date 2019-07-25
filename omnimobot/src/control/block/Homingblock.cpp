#include <omnimobot/control/block/Homingblock.hpp>

using namespace eeros::control;
using namespace omnimobot;

HomingBlock::HomingBlock() : 
swWeel1(1),			// Switch is connect with zerotorque
swWeel2(1),
swWeel3(1),
swSteer1(1),
swSteer2(1),
swSteer3(1),
zeroTorque(0.0)
{
	getInTorque().getSignal().setName("controlled Torque");
	getOutHomingTorque().getSignal().setName("homing torque");
	
	// Switch with controled torque (Demux) connect
	swWeel1.getIn(0).connect(deMuxTorque.getOut(0));
	swWeel2.getIn(0).connect(deMuxTorque.getOut(1));
	swWeel3.getIn(0).connect(deMuxTorque.getOut(2));
	swSteer1.getIn(0).connect(deMuxTorque.getOut(3));
	swSteer2.getIn(0).connect(deMuxTorque.getOut(4));
	swSteer3.getIn(0).connect(deMuxTorque.getOut(5));
	
	// Switch with constante connect
	swWeel1.getIn(1).connect(zeroTorque.getOut());
	swWeel2.getIn(1).connect(zeroTorque.getOut());
	swWeel3.getIn(1).connect(zeroTorque.getOut());
	swSteer1.getIn(1).connect(zeroTorque.getOut());
	swSteer2.getIn(1).connect(zeroTorque.getOut());
	swSteer3.getIn(1).connect(zeroTorque.getOut());
	
	muxTorque.getIn(0).connect(swWeel1.getOut());
	muxTorque.getIn(1).connect(swWeel2.getOut());
	muxTorque.getIn(2).connect(swWeel3.getOut());
	muxTorque.getIn(3).connect(swSteer1.getOut());
	muxTorque.getIn(4).connect(swSteer2.getOut());
	muxTorque.getIn(5).connect(swSteer3.getOut());
	
};

void HomingBlock::run(){
	deMuxTorque.run();
	zeroTorque.run();
	swWeel1.run();
	swWeel2.run();
	swWeel3.run();
	swSteer1.run();
	swSteer2.run();
	swSteer3.run();
	muxTorque.run();
	
}

HomingBlock::~HomingBlock()
{

}

eeros::control::Input<Vector6>& HomingBlock::getInTorque() {
	return deMuxTorque.getIn();
}

eeros::control::Output<Vector6>& HomingBlock::getOutHomingTorque() {
	return muxTorque.getOut();
}