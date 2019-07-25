#ifndef __CH_NTB_OMNIMOBOT_LASERSCANNER_HPP
#define __CH_NTB_OMNIMOBOT_LASERSCANNER_HPP

#include "CommandURG04LX.hpp"
#include <eeros/math/Matrix.hpp>

/**********************************************************
 * File:     LaserScanner.hpp                              
 * Created:  2014  S.Landis                      
 * Changes:	 none
 * ------------------------------------------------------ 
 * Description:                                           
 * Creates a Datamatrix with the distance [m] and the angle [rad] values 
 * from the Laserscanner URG04LX. The scanning area can be specified.
 * It delegated and control the scanning                                                                  
 **********************************************************/
			

#define MAX_MATRIX_ROW 682	// 725-44 (1 weil 44 auch zählt) fängt bei 0 an

typedef eeros::math::Matrix<MAX_MATRIX_ROW,2> DataMatrix;


class LaserScanner {

public:
	LaserScanner();
	LaserScanner(int startS, int endS);
	virtual ~LaserScanner();
	
	virtual bool openScanner(const char* dev);
	virtual void closeScanner();
	virtual bool scan();
	virtual DataMatrix& getScannData();
	
	/** Using this method, the scanning area can be specified
	* @param start	data in step (first = 44)
	* @param end	data in step (last = 725)
	*/
	virtual void setStartStepAndEndStep(int start, int end);
	
	
	virtual int& getOutDataSize();

	
private: 
	int sendData(const char* str, int strSize);
	bool readData(char* buffer, char* startOfSequence);
	
	virtual bool initScanner();
	virtual bool decode(const char* bufferPoint);
	
	static constexpr int numberOfLfBeforeData = 6; 
	static constexpr int lengthOfStartsequence = 2;	// Number of characters to be tested
	static constexpr int importantLf = 6; 
	static constexpr int numberOfByteInDataPacket = 64; 
	static constexpr int countsBeforeData = 47;

	static constexpr int minDistance = 20;
	static constexpr int maxDistance = 5600;
	static constexpr int frontStep = 384;
	static constexpr int totalSteps = 1024;

	static constexpr int bufferSize = 4096;
	char buffer[bufferSize];
	
	int readSize;
	int desired_lf_count;
	int startStep;
	int endStep;
	int fd;
	int dataSize;
	int matrixPos;
	int step_count;
	int desired_countData;

	static constexpr double angleResulution = (2.0*M_PI)/totalSteps;		// to rad
// 	static constexpr double angleResulution = 360.0/totalSteps;		// to grad

	DataMatrix distanceAndAngle;
	
	ScanCommand scanCommand;
};


#endif /* __CH_NTB_OMNIMOBOT_LASERSCANNER_HPP */