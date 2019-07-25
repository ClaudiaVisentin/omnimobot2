#include <LSData.hpp>
#include <iostream>

LSData::LSData():
dataSize(1),
startStep(44),
endStep(725)
{
	distanceAndAngle.zero();
}

LSData::LSData(int startS, int endS):
dataSize(1),
startStep(startS),
endStep(endS)
{
	distanceAndAngle.zero();
}

LSData::~LSData()
{

}


bool LSData::decode(const char* str, int strSize)
{
	int matrixPos = 0;
	
	int step_count = startStep;
	int desired_countData = ((endStep - startStep + 1) * 3); // +1 because the step 44 is also output. at MD -> *3, at MS -> *2. 

	int desiredStrSize = desired_countData + COUNTS_BEFORE_DATA;
	int error = 0;
	int char_count = 0;
	int char_countData = 0;
	int lf_count = 0;
	int count_64 = 0;
	int i = 0;
	
	double angle_count = 0.0;
	double angleResulution = (2.0*M_PI)/totalSteps;		// to rad
//	double angleResulution = 360.0/totalSteps;		// to grad
	
	unsigned char c0 = 0;
	unsigned char c1 = 0;
	unsigned char c2 = 0;
	
	bool first = true;
	bool first_count = true;
	
	if(strSize < desiredStrSize) {
		std::cout << "wrong string size " << strSize << " < " << desiredStrSize << std::endl;
		return false;
	}
	
	// process char_countData
	while(str[char_count] != 0 && char_countData <= desired_countData) {
		char count = 0;
		
		count = str[char_count];
		
		if (first_count){
			if(count != 'M'){													
			throw -1; //::cout << "ERROR " << count << std::endl;
			return false;
			}
			else {
				first_count = false;
			}
		}
		
		if (count == '\n'){		// counts number LF
			lf_count++;
		}
		
		if(lf_count == IMPORTANT_LF && first){		// After the 6th data come
			count_64 = 0;
			first = false;
		}
		
		if (lf_count >= IMPORTANT_LF && count_64 >=1 && count_64 <= NUMBER_OF_BYTE_IN_DATA_PACKET && char_countData <= desired_countData ){		// nach jeweils 64 byts kommt ein byt sum und ein LF (diese sollen heraus gefiltert werden)
			
// 			std::cout << (char)count << " -> countData: " << char_countData  << " -> count64: " << count_64 << std::endl;
			
			// converting data in distances
			if (i == 0){
				c0 = (char)count - '0'; //  30H is subtracted
				i = 1;
			}
			else if(i == 1){
				c1 = (char)count - '0';
				i = 2;
			}
			else {
				c2 = (char)count - '0';
				i = 0;
			}
			
			if ( i == 0){
				int distance = ((c0 & 0x3f) << 12) | ((c1 & 0x3f) << 6) | (c2 & 0x3f); 
			
				if (distance <= minDistance){
					distance = maxDistance;
				}
				if (distance >= maxDistance){
					
					distance = maxDistance;
				}
//  				std::cout << "distance: " << distance << " Step_count: " << step_count << " matrixPos: " <<matrixPos << std::endl;
				
				distanceAndAngle(matrixPos,0) =  static_cast<double>(distance) / 1000.0;		//1000 (to meter)
				
				// Winkel in rad
				if (step_count >= frontStep) {
					angle_count = (step_count - frontStep)*angleResulution;
				}
				else {
					angle_count = -(frontStep - step_count )*angleResulution; // negativ wenn nach rechts
				}
				
				distanceAndAngle(matrixPos,1) =	static_cast<double>(angle_count);
				
//   				std::cout << "distance: " << distanceAndAngle(matrixPos,0) << " Angle " << distanceAndAngle(matrixPos,1) << std::endl;
				
				
				step_count++;
				matrixPos++;
			}

			char_countData++;
		}
		
		count_64++;
		
		// count_64 reset
		if (lf_count >= IMPORTANT_LF && count_64 == NUMBER_OF_BYTE_IN_DATA_PACKET+2){
			count_64 = 0;
		}
		char_count++;
	}
	
	dataSize = matrixPos; 
	
	return true;
}


dataMatrix& LSData::getData()
{
	
	return distanceAndAngle;

}


int LSData::getDataSize()
{
	return dataSize;
}


int LSData::getEndStep()
{
	return endStep;
}

int LSData::getMaxPossibleDistance()
{
	return maxDistance;
}

int LSData::getMinPossibleDistance()
{
	return minDistance;
}

int LSData::getStartStep()
{
	return startStep;
}

int LSData::getTotalSteps()
{
	return totalSteps;
}

void LSData::setStartAndEndStep(int start, int end)
{
	startStep = start;
	endStep = end;
}

