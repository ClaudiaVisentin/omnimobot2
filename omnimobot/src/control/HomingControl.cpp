#include <omnimobot/control/HomingControl.hpp>

using namespace eeros::control;
using namespace omnimobot;

HomingControl::HomingControl() :
	setvelo(0),			
	q("q"),				
	kv(500.0),
	inertia(0.06672),
	invgear(0.027211),		// 1/36.75
	invMotConst(33.1126),	// 1/0.0302
	ohmMot(0.299),
	gearMotconst(1.10985), 	// 36.75*0.0302
	voltage("volt"),		
	executor(0.001) 
{
	sum1.negateInput(1);
	
	diff.getIn().connect(q.getOut());
	sum1.getIn(0).connect(setvelo.getOut());
	sum1.getIn(1).connect(diff.getOut());
	kv.getIn().connect(sum1.getOut());
	invgear.getIn().connect(kv.getOut());
	inertia.getIn().connect(invgear.getOut());
	ohmMot.getIn().connect(inertia.getOut());
	invMotConst.getIn().connect(ohmMot.getOut());
	gearMotconst.getIn().connect(diff.getOut());
	sum2.getIn(0).connect(invMotConst.getOut());
	sum2.getIn(1).connect(gearMotconst.getOut());
	voltage.getIn().connect(sum2.getOut());
	
	executor.addRunnable(this);
	
}

void HomingControl::run() {
	setvelo.run();
	q.run();
	diff.run();
	sum1.run();
	kv.run();
	invgear.run();
	inertia.run();
	ohmMot.run();
	invMotConst.run();
	gearMotconst.run();
	sum2.run();
	voltage.run();
}

void HomingControl::start() {
	executor.start();
}

void HomingControl::stop() {
	executor.stop();
}


HomingControl& HomingControl::instance() {	
	static HomingControl homingControlInstance;
	return homingControlInstance;
}

