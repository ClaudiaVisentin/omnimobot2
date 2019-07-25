#ifndef __CH_NTB_OMNIMOBOT_COMMANDURG04LX_HPP
#define __CH_NTB_OMNIMOBOT_COMMANDURG04LX_HPP

#include <array>
#include <sstream>
#include <unistd.h>

#define BUFFERTEST 15

// Diese Klasse bereitet aus einem vorgegebenen Bereich des Laserscanners das passende Commando vor

template<int L>
class CommandURG04LX {
public:
	
	CommandURG04LX(const char* str) {
        int i = 0;
        while(str[i] != 0 && i < L) {
            cmd[i] = str[i];
            i++;
       }
       cmd[i] = 0;
    }
	
	bool insertValue(int value, unsigned int digits, unsigned int index) {
        if(index + digits > L || value < 0 || value > power(10, digits) - 1) return false;
        for(int i = 0; i < digits; i++) {
            cmd[index + digits - 1 - i] = value % 10 + '0';
            value = value / 10;
        }
    }
    
    const char operator[](int i) const {
        if(i >= 0 && i < L) {
            return cmd[i];
        }
        else {
            throw -123456789;
        }
    }
    
    char* toCString() {
      
        return cmd;
    }
    
    bool check(const char* buffer){
		
		for (int i=0; i < BUFFERTEST; i++){
			if(buffer[i] == cmd[i]){
				continue;
			}
			else {
				return false;
			}
		}
		return true;
	}

protected:
    char cmd[L];
	
private:
	
	
	int power(int base, int exp) {
		int result = 1;
		while(exp) {
			result *= base;
			exp--;
		}
		return result;
	}

};

class ScanCommand : public CommandURG04LX<17> {
public:
    ScanCommand() : CommandURG04LX("MD0000000000001\n") { }
					
    void setStartValue(int start){
		insertValue(start, 4, 2);
	}
	
	void setEndValue(int end){
		insertValue(end, 4, 6);
	}

};



template<int L>
std::ostream& operator<<(std::ostream& os, const CommandURG04LX<L>& right) {
    for(int i = 0; i < L; i++) {
        os << right[i];
    }
    return os;
};

#endif /* __CH_NTB_OMNIMOBOT_COMMANDURG04LX_HPP */