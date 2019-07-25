#ifndef __CH_NTB_OMNIMOBOT_CONTROL_LASERSCANNERCONTROL_HPP
#define __CH_NTB_OMNIMOBOT_CONTROL_LASERSCANNERCONTROL_HPP

#include <eeros/core/Runnable.hpp>
#include <eeros/math/Frame.hpp>
#include <eeros/control/TimeDomain.hpp>
#include <omnimobot/control/block/LaserScannerblock.hpp>
#include <omnimobot/control/block/TransitionLaserData.hpp>

#include <types.hpp>


namespace omnimobot
{
	class LaserScannerControl {
		
	public:
		LaserScannerControl(double ts);
 		virtual ~LaserScannerControl();
		
		void start();
		void stop();

		LaserScannerblock laserBlock;
		TransitionLaserData transDataBlock;
		
		eeros::control::TimeDomain timedomain;

		
	private:
		

	};
}

#endif /* __CH_NTB_OMNIMOBOT_CONTROL_LASERSCANNERCONTROL_HPP */