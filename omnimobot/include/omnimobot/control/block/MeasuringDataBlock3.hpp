#ifndef __CH_NTB_OMNIMOBOT_CONTROL_BLOCK_MEASURINGDATABLOCK3_HPP
#define __CH_NTB_OMNIMOBOT_CONTROL_BLOCK_MEASURINGDATABLOCK3_HPP

#include <omnimobot/constants.hpp>
#include <eeros/control/Block1i1o.hpp>
// #include <eeros/core/System.hpp>
#include <eeros/math/Matrix.hpp>
#include <eeros/control/Block.hpp>
#include <eeros/control/Input.hpp>


namespace omnimobot 
{

	class MeasuringDataBlock3 : public eeros::control::Block
	{
		
	public:
		MeasuringDataBlock3();
		virtual ~MeasuringDataBlock3();
		
		virtual eeros::control::Input<eeros::math::Vector<3>>& getIn() {
			return in;
		}
		
		virtual void startLog();
// 		virtual void resetLog();
		virtual bool getMeasuringData(double* bufferValue0, double* bufferValue1  );
		
		virtual void run();


	protected:
		eeros::control::Input<eeros::math::Vector<3>> in;

		
	private:	
		int i;
		double buffer0[sizeDatalog];
		double buffer1[sizeDatalog];
		bool log;
		
	};
};

#endif /* __CH_NTB_OMNIMOBOT_CONTROL_BLOCK_MEASURINGDATABLOCK3_HPP */