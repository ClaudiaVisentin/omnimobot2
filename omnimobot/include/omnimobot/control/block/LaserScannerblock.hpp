#ifndef __CH_NTB_OMNIMOBOT_CONTROL_BLOCK_LASERSCANNERBLOCK_HPP
#define __CH_NTB_OMNIMOBOT_CONTROL_BLOCK_LASERSCANNERBLOCK_HPP

#include <omnimobot/constants.hpp>
#include <eeros/math/Matrix.hpp>
#include <eeros/control/Block.hpp>
#include <eeros/control/Output.hpp>
#include "../../../../laserscanner/include/LaserScanner.hpp"
#include "../../../../laserscanner/include/CollisionDetection.hpp"
#include <eeros/core/System.hpp>


namespace omnimobot
{

	class LaserScannerblock : public eeros::control::Block
	{
	public:
		LaserScannerblock(std::string dev);
		virtual ~LaserScannerblock();
			
		virtual eeros::control::Output<CollisionDataMatrix>& getOutColldata() {
			return outColldata;
		}

		virtual void run();


	private:
	
		eeros::control::Output<CollisionDataMatrix> outColldata;
		
		CollisionDataMatrix colldata;
		
		LaserScanner laserScanner;
		CollisionDetection collDedection;

	};
	
}
#endif /* __CH_NTB_OMNIMOBOT_CONTROL_BLOCK_LASERSCANNERBLOCK_HPP */
