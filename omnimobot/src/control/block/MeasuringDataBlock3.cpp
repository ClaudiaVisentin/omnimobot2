#include <omnimobot/control/block/MeasuringDataBlock3.hpp>
#include <omnimobot/constants.hpp>

#include <iostream>

using namespace omnimobot;
using namespace eeros::math;
using namespace std;

MeasuringDataBlock3::MeasuringDataBlock3():
log(false),
i(0)
{ 
// 	buffer.zero();
}

MeasuringDataBlock3::~MeasuringDataBlock3() { }

void MeasuringDataBlock3::run()
{
	if(log) {
		if ( i < sizeDatalog ){
			buffer0[i] = in.getSignal().getValue()(0);
			buffer1[i] = in.getSignal().getValue()(1);
			
			if (i == sizeDatalog-1) {
				std::cout << "measuring ended "  << std::endl;
			}
			i++;
		}
		else {
			log = false;
		}
	}
}

void MeasuringDataBlock3::startLog()
{
// 	std::cout << "measuring start"  << std::endl;
	i = 0;
	log = true;
	
}

// void MeasuringDataBlock2::resetLog()
// {
// 	log = false;
// 	i = 0;
// }

bool MeasuringDataBlock3::getMeasuringData(double* bufferValue0, double* bufferValue1 )
{
	for(int j = 0; j < sizeDatalog; j++){
		bufferValue0[j] = buffer0[j];
		bufferValue1[j] = buffer1[j];
	}
		
	return true;
}
