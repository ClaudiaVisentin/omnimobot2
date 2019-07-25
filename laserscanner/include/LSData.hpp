#ifndef __CH_NTB_OMNIMOBOT_LSDATA_HPP
#define __CH_NTB_OMNIMOBOT_LSDATA_HPP

#include <eeros/math/Matrix.hpp>

#define MAX_MATRIX_ROW 682
#define IMPORTANT_LF 6
#define NUMBER_OF_BYTE_IN_DATA_PACKET 64
#define FOURTH_COUNT 4
#define COUNTS_BEFORE_DATA 47

typedef eeros::math::Matrix<MAX_MATRIX_ROW,2> dataMatrix;


class LSData {

public:
	LSData();
	LSData(int startS, int endS);
	virtual ~LSData();
	
	virtual bool decode(const char* str, int bufferSize);
	virtual dataMatrix& getData();
	
	virtual int getMinPossibleDistance();
	virtual int getMaxPossibleDistance();
	virtual int getTotalSteps();
	virtual int getStartStep(); 
	virtual int getEndStep();
	virtual int getDataSize();
	
	virtual void setStartAndEndStep( int start, int end); // only the range is not 44-725 steps	
	
private: 
	
	static constexpr int minDistance = 20;
	static constexpr int maxDistance = 5600;
	static constexpr int frontStep = 384;
	static constexpr int totalSteps = 1024;
	
	int dataSize;
	int startStep;
	int endStep;
	
	dataMatrix distanceAndAngle;
};




#endif /* __CH_NTB_OMNIMOBOT_LSDATA_HPP */