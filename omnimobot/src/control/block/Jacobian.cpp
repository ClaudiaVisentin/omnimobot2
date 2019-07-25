#include <omnimobot/control/block/Jacobian.hpp>


using namespace omnimobot;
using namespace eeros::math;


Jacobian::Jacobian():
jacobi()
{
	omega_w.zero();
	v_L.zero();
	v_L_Out.zero();
	phi_j.zero();
}

Jacobian::~Jacobian() { }

void Jacobian::run()
{
	omega_w = inOmega.getSignal().getValue();
	phi_j(0) = inActualPosQ.getSignal().getValue()(3);
	phi_j(1) = inActualPosQ.getSignal().getValue()(4);
	phi_j(2) = inActualPosQ.getSignal().getValue()(5);
		
	v_L_Out = jacobi.getVeloLocal(omega_w, v_L, phi_j); 

	outPhiGLd.getSignal().setValue(v_L_Out(2));
	outPhiGLd.getSignal().setTimestamp(inOmega.getSignal().getTimestamp());
	outV_L.getSignal().setValue(v_L_Out);
	outV_L.getSignal().setTimestamp(inOmega.getSignal().getTimestamp());

}
