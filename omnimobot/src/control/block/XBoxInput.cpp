#include <omnimobot/control/block/XBoxInput.hpp>
#include <omnimobot/XBoxController.hpp>
#include <omnimobot/constants.hpp>
#include <iostream>

using namespace omnimobot;
using namespace remote;
using namespace eeros::control;
using namespace eeros::math;

XBoxInput::XBoxInput(std::string dev): 
LButton(0),
RButton(0),
xyScale(0.0),
rotScale(0.0),
RTactual(0.0),
LTactual(0.0),
saturation(0.002), //  [m/s] 
saturationRot(0.1), // [rad/s]
stopDrive(false),
rot(0.0)

{
	j.open(dev.c_str());
	t = new std::thread([this](){ this->j.loop(); });
	
	vOut.zero();
	vOutPrev.zero();
	in.zero();
	direction.zero();
}

XBoxInput::~XBoxInput() {
	delete t;
	j.close();
}

void XBoxInput::run() {

	RTactual = j.current.axis[XBoxController::Axis::RT];
	LTactual = j.current.axis[XBoxController::Axis::LT];
	
	LButton = j.current.button_state[XBoxController::Button::L];
	RButton = j.current.button_state[XBoxController::Button::R];
	
	// Input
	in <<  j.current.axis[XBoxController::Axis::RX],
	       j.current.axis[XBoxController::Axis::RY];
	
	vOutPrev << vIn.getSignal().getValue()(0),
				vIn.getSignal().getValue()(1),
				0.0,
				vIn.getSignal().getValue()(2),
				
	state = j.current;
	
	
	// Input control	  
	for(int i = 0; i < 2; i++) {
		if(in(i) > -0.2 && in(i) < 0.2 && i != 2) {
			in(i) = 0;
		}
	}
			
  
	// Velocity xy
	// RTactual
	if (RTactual == -1.0){
		xyScale = 0.0;
		
		for (int i = 0; i < 2; i++){
			vOut(i) = 0.0;
		}
	}
	else if(in(0) == 0 && in(1) == 0) {
		xyScale = 0.0;
		
		for (int i = 0; i < 2; i++){
			vOut(i) = 0.0;
		}
	}
	else {
		
		xyScale = (RTactual + 1.0) / 2.0; // amount of speed
		
		// Vector of the Velocity
		for (int i = 0; i < 2; i++){
			direction(i) = in(i) / sqrt(in(0)*in(0) + in(1)*in(1));
			
			vOut(i) = xyScale * direction(i); // [m/s]
			
// 			double vOutabsTmp = sqrt(vOut(i)*vOut(i));
			
			// ramp for the start
			if (vOut(i) > 0.01 && vOutPrev(i) >= 0.0 && vOut(i) > vOutPrev(i) && vOut(i) - vOutPrev(i) > saturation ) { // saturation[m/s] saturation
				vOut(i) = 	vOutPrev(i) + saturation;
			}
			else if (vOut(i) < -0.01 && vOutPrev(i) <= 0.0 && vOut(i) < vOutPrev(i) && vOut(i) - vOutPrev(i) < -saturation){
				vOut(i) = 	vOutPrev(i) - saturation;
			}
		}
	}
	
			
	// Rotation

	rotScale = (LTactual + 1.0) / 2.0;
	
	// Vector of the Velocity
	if(LButton == 1 ) {
		
		rot = -rotScale;
	}
	else if (RButton == 1){
		
		rot = rotScale;
	}
	else {
		rot = 0.0;
	}
	
	// stop is active
	if(stopDrive){
		for (int i = 0; i < 2; i++){
			vOut(i) = 0.0;
		}
		rot = 0.0;
	}
	
	vOut(3) = rot; // [rad]

	// ramp for rot
	if (vOut(3) > 0.01 && vOutPrev(3) >= 0.0 && vOut(3) > vOutPrev(3) && vOut(3) - vOutPrev(3) > saturationRot ) { // saturationRot [m/s] saturation
		vOut(3) = 	vOutPrev(3) + saturationRot;
	}
	else if (vOut(3) < -0.01 && vOutPrev(3) <= 0.0 && vOut(3) < vOutPrev(3) && vOut(3) - vOutPrev(3) < -saturationRot){
		vOut(3) = 	vOutPrev(3) - saturationRot;
	}
	
	out.getSignal().setValue(vOut); 
	
	uint64_t ts = eeros::System::getTimeNs();
	out.getSignal().setTimestamp(ts);

}


bool XBoxInput::isStop()
{
	stopDrive = j.current.button_down[XBoxController::Button::B];
	return stopDrive;
}

bool XBoxInput::startMeasuring()
{
	return j.current.button_down[XBoxController::Button::A];
}

bool XBoxInput::shoutDown()
{
	return j.current.button_down[XBoxController::Button::X];
}



