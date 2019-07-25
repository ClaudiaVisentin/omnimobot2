#include <omnimobot/control/block/InvJacobian.hpp>


using namespace omnimobot;
using namespace eeros::math;


InvJacobian::InvJacobian():
phi_GL(0.0),
invjacobi()
{ 
	v_GL.zero();
	omega_w.zero();
	phi_ij.zero();
	omega.zero();
	omega_steer.zero();
}

InvJacobian::~InvJacobian() { }

void InvJacobian::run()
{
	
	v_GL = in_V_global.getSignal().getValue();
	phi_ij = inPhiSteer.getSignal().getValue();
	phi_GL =  phi_Global.getSignal().getValue();

	omega = invjacobi.getOmega(omega_w, v_GL,phi_ij,phi_GL);
	
	omega_steer(0) = omega(3);
	omega_steer(1) = omega(4);
	omega_steer(2) = omega(5);

	outOmega.getSignal().setValue(omega);
	outOmega.getSignal().setTimestamp(inPhiSteer.getSignal().getTimestamp());
	outOmegaSteer.getSignal().setValue(omega_steer);
	outOmegaSteer.getSignal().setTimestamp(inPhiSteer.getSignal().getTimestamp());
}
