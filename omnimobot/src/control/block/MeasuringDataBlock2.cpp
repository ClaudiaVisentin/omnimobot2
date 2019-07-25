#include <omnimobot/control/block/MeasuringDataBlock2.hpp>
#include <omnimobot/constants.hpp>

#include <iostream>

using namespace omnimobot;
using namespace eeros::math;
using namespace std;

MeasuringDataBlock2::MeasuringDataBlock2():
log(false),
i(0)
{ }

MeasuringDataBlock2::~MeasuringDataBlock2() { }

void MeasuringDataBlock2::run()
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

void MeasuringDataBlock2::startLog()
{
	i = 0;
	log = true;
}


bool MeasuringDataBlock2::getMeasuringData(double* bufferValue0, double* bufferValue1 )
{
	for(int j = 0; j < sizeDatalog; j++){
		bufferValue0[j] = buffer0[j];
		bufferValue1[j] = buffer1[j];
	}
		
	return true;
}
