#include <LaserScanner.hpp>

#include <stdio.h>
#include <signal.h>
#include <fcntl.h>
#include <termios.h>

LaserScanner::LaserScanner():
readSize(0),
desired_lf_count(0),
startStep(44),
endStep(725),
dataSize(1),
matrixPos(0),
step_count(0),
desired_countData(0)
{
	scanCommand.setStartValue(startStep);
	scanCommand.setEndValue(endStep);
	distanceAndAngle.zero();
	initScanner();
	//fd = ::open(dev, O_RDWR | O_NONBLOCK  | O_FSYNC); // init
}

LaserScanner::LaserScanner(int startS, int endS):
readSize(0),
desired_lf_count(0),
startStep(startS),
endStep(endS),
dataSize(1),
matrixPos(0),
step_count(0),
desired_countData(0)
{
	if(startStep < 44){
		startStep = 44;
	}
	
	if(endStep > 725){
		endStep = 725;
	}
	scanCommand.setStartValue(startStep);
	scanCommand.setEndValue(endStep);
	distanceAndAngle.zero();
	initScanner();
}


LaserScanner::~LaserScanner()
{
	closeScanner();
}



bool LaserScanner::openScanner(const char* dev)
{
	speed_t baud = B115200; /* baud rate */
	
	fd = ::open(dev, O_RDWR | O_NONBLOCK  | O_FSYNC); // init
	
	/* set the other settings (in this case, 115200 8N1) */
	struct termios settings;
	tcgetattr(fd, &settings);

	cfsetospeed(&settings, baud); /* baud rate */
	settings.c_cflag &= ~PARENB; /* no parity */
	settings.c_cflag &= ~CSTOPB; /* 1 stop bit */
	settings.c_cflag &= ~CSIZE;
	settings.c_cflag |= CS8 | CLOCAL; /* 8 bits */
//	settings.c_lflag = ICANON; /* canonical mode */
	settings.c_lflag &=(~ICANON & ~ECHO);
	settings.c_oflag &= ~OPOST; /* raw output */
	settings.c_iflag &= ~INPCK;

	tcsetattr(fd, TCSANOW, &settings); /* apply the settings */
	tcflush(fd, TCOFLUSH);

	return fd;
}



void LaserScanner::closeScanner()
{
	::close(fd);
}



bool LaserScanner::initScanner()
{
	desired_countData = ((endStep - startStep + 1) * 3); // +1 weil der Step 44 auch ausgegeben wird. bei MD -> *3 bei MS -> *2. 
	
	int numberOfDatablocks = desired_countData / numberOfByteInDataPacket;
	
	if(desired_countData % numberOfByteInDataPacket != 0){
		numberOfDatablocks = numberOfDatablocks+1;
	}
	
	readSize = numberOfDatablocks * 2 + desired_countData + countsBeforeData + 2;	// +2: +1 wegen 0 terminiert und +1 mit letztes LF
	
	desired_lf_count = numberOfLfBeforeData + numberOfDatablocks +1;
	
	return true;
}



bool LaserScanner::scan()
{
	if(!readSize == 0){
	
		sendData(scanCommand.toCString(), 17);		// scanCommand had 17 counts 

		// read data
		if(readSize < bufferSize - 1) {
			
			
			if(!readData(buffer, scanCommand.toCString())){
				throw -1; // TODO
				return false;
			}
			buffer[readSize] = 0;
		}
		else {
			throw -2; // TODO
			return false;
		}
		
		// decode data
		if(decode(buffer)){
			return true;
		}
		else{
			throw -3; // TODO
			return false;
		}
	}
	else{
		throw -4; // TODO
		return false;
	}
}



int LaserScanner::sendData(const char* str, int strSize)
{
	return write(fd,str,strSize);
}



bool LaserScanner::readData(char* buffer,  char* startOfSequence)
{
	bool check = false;

	// find start of sequence
	do{
		if(!check){
			int numberCountsInBuffer = 0;
			int timeout = 0;
			do{// read data with timeout
				numberCountsInBuffer = read(fd, buffer, lengthOfStartsequence);
				if (numberCountsInBuffer < lengthOfStartsequence) usleep(1000);
				if (timeout > 1000) return false;
				timeout++;
			}while(numberCountsInBuffer < lengthOfStartsequence);
		}

		for(int i = 0; i < lengthOfStartsequence; i++){
			if(buffer[i]!=startOfSequence[i]){
				check = false;
				break;
			}
			if(i==lengthOfStartsequence-1){
				check =true;
				break;
 			}
		}
	}while(!check);
	
	// insert after start of sequence
	buffer = buffer+ lengthOfStartsequence;

	int lf_count = 0;
	
	// read date and number lf counts
	for(int j = 0; lf_count < desired_lf_count && j < readSize-lengthOfStartsequence-1; j++){ 
		
		int numberCountsInBuffer = 0;
		int timeout = 0;
		
		do{// read data with timeout
			numberCountsInBuffer = read(fd, buffer, 1);
			if (numberCountsInBuffer < 1) usleep(1000);
			if (timeout > 1000) return false;
			timeout++;
		}while(numberCountsInBuffer < 1);

		if(*buffer == '\n'){
			lf_count++;
		}
		buffer++;
	} 
	
	return true;
}




bool LaserScanner::decode(const char* bufferPoint)
{
	
	matrixPos = 0;
	
	step_count = startStep;

	int error = 0;
	int char_count = 0;
	int char_countData = 0;
	int lf_count = 0;
	int count_64 = 0;
	int i = 0;
	
	double angle_count = 0.0;

	unsigned char c0 = 0;
	unsigned char c1 = 0;
	unsigned char c2 = 0;
	
	bool first = true;
	bool first_count = true;
	
	// process char_countData
	while(bufferPoint[char_count] != 0 && char_countData <= desired_countData) {
		char count = 0;
		
		count = bufferPoint[char_count];
		
		if (first_count){
			if(count != 'M'){													
			throw -1; 
			return false;
			}
			else {
				first_count = false;
			}
		}
		
		if (count == '\n'){		// counts number LF
			lf_count++;
		}
		
		if(lf_count == importantLf && first){		// After the 6th data come
			count_64 = 0;
			first = false;
		}
		
		if (lf_count >= importantLf && count_64 >=1 && count_64 <= numberOfByteInDataPacket && char_countData <= desired_countData ){		// nach jeweils 64 byts kommt ein byt sum und ein LF (diese sollen heraus gefiltert werden)

			
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
				int distance = ((c0 & 0x3f) << 12) | ((c1 & 0x3f) << 6) | (c2 & 0x3f); // c0 und 00111111 gibt richtiges Bitmuster und schiebt es 12 plätze rüber 111100000000000000 |(oder) werden oder verknüpft mit dem nächsten Bitmuster
			
				if (distance <= minDistance){
					distance = maxDistance;
				}
				if (distance >= maxDistance){
					
					distance = maxDistance;
				}
				
				distanceAndAngle(matrixPos,0) =  static_cast<double>(distance) / 1000.0;		//1000 (to meter)
				
				// Winkel in rad
				if (step_count >= frontStep) {
					angle_count = (step_count - frontStep)*angleResulution;
				}
				else {
					angle_count = -(frontStep - step_count )*angleResulution; // negativ wenn nach rechts
				}
				
				distanceAndAngle(matrixPos,1) =	static_cast<double>(angle_count);
				
				step_count++;
				matrixPos++;
			}
			char_countData++;
		}
		
		count_64++;
		
		// count_64 reset
		if (lf_count >= importantLf && count_64 == numberOfByteInDataPacket+2){
			count_64 = 0;
		}
		char_count++;
	}
	
	dataSize = matrixPos; 
	return true;
	
}



DataMatrix& LaserScanner::getScannData()
{
	return distanceAndAngle;
}



void LaserScanner::setStartStepAndEndStep(int start, int end)
{
	startStep = start;
	endStep = end;
	
	if(startStep < 44){
		startStep = 44;
	}
	
	if(endStep > 725){
		endStep = 725;
	}
	
	scanCommand.setStartValue(startStep);
	scanCommand.setEndValue(endStep);
	
	initScanner();
	
}



int& LaserScanner::getOutDataSize()
{
	return dataSize;
}
