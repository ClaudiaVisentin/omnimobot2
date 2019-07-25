#include <omnimobot/control/block/PendulumControl.hpp>
#include <omnimobot/constants.hpp>
#include <cmath>

using namespace omnimobot;
using namespace eeros::math;
using namespace eeros::control;
using namespace std;


PendulumControl::PendulumControl():
aPhi_To_axy(),
phi_To_Tip(),
gKpPos(kpTip),
gKdPos(kdTip),
gKpPhi(kpPhi),
gKdPhi(kdPhi),
inActual(1.0),
inDesired(1.0),
phixyIn(1.0)		// gate
{
	getInPosTip().getSignal().setName("PosTipDesired xdes,ydes");
	getInOdometry().getSignal().setName("odometrie of the Robot x,y,theta");
	getInPhixy().getSignal().setName("Angle Phi from Hall-Sensoren");
	getOutVelo().getSignal().setName("VeloLokal vx,vy");
	

 	phi_To_Tip.getInPhixy().connect(phixyIn.getOut());
	phi_To_Tip.getInOdometry().connect(inActual.getOut());
	SumTip.negateInput(1);
	SumTip.getIn(0).connect(inDesired.getOut());
	SumTip.getIn(1).connect(phi_To_Tip.getOutTip());
	gKpPos.getIn().connect(SumTip.getOut());
	diffTip.getIn().connect(SumTip.getOut());
	gKdPos.getIn().connect(diffTip.getOut());
	SumPDTip.getIn(0).connect(gKdPos.getOut());
	SumPDTip.getIn(1).connect(gKpPos.getOut());
	aTip_To_Phi.getInAccTip().connect(SumPDTip.getOut());
	SumPhi.negateInput(1);
	SumPhi.getIn(0).connect(aTip_To_Phi.getOutPhiSoll());
	SumPhi.getIn(1).connect(phixyIn.getOut());
	gKpPhi.getIn().connect(SumPhi.getOut());
	diffPhi.getIn().connect(SumPhi.getOut());
	gKdPhi.getIn().connect(diffPhi.getOut());
	SumPDPhi.getIn(0).connect(gKdPhi.getOut());
	SumPDPhi.getIn(1).connect(gKpPhi.getOut());
	aPhi_To_axy.getInAccPhi().connect(SumPDPhi.getOut());
	aPhi_To_axy.getInPhixy().connect(phixyIn.getOut()); 
	integrator.getIn().connect(aPhi_To_axy.getOutAccxy());
	
	measuringTipActual.getIn().connect(phi_To_Tip.getOutTip()); // only for test
	measuringTipDes.getIn().connect(inDesired.getOut()); // only for test
	measuringPosActual.getIn().connect(inActual.getOut()); // only for test
	
}


void PendulumControl::run(){
	phixyIn.run();
	inActual.run(); 
	inDesired.run(); 
	phi_To_Tip.run();
	SumTip.run();
	gKpPos.run();
	diffTip.run();
	gKdPos.run();
	SumPDTip.run();
	aTip_To_Phi.run();
	SumPhi.run();
	gKpPhi.run();
	diffPhi.run();
	gKdPhi.run();
	SumPDPhi.run();
	aPhi_To_axy.run();
	integrator.run();
	
	measuringTipActual.run();
	measuringTipDes.run();
	measuringPosActual.run();
}

Input< Vector2 >& PendulumControl::getInPhixy()
{
	return phixyIn.getIn();
}

Input< Vector3 >& PendulumControl::getInOdometry()
{
	return inActual.getIn();
}

Input< Vector2 >& PendulumControl::getInPosTip()
{
	return inDesired.getIn();
}

Output< Vector2 >& PendulumControl::getOutVelo()
{
	return integrator.getOut();
}
