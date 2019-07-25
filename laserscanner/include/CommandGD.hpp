#ifndef __CH_NTB_OMNIMOBOT_COMMANDGD_HPP
#define __CH_NTB_OMNIMOBOT_COMMANDGD_HPP

#include <array>
#include <iostream>
#include <sstream>
#include <unistd.h>

template<int L>
class CommandGD {
public:
	
	CommandGD(const char* str) {
        int i = 0;
        while(str[i] != 0 && i < L) {
            cmd[i] = str[i];
            i++;
        }
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
    
    
    std::array<char, L> cmd;
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

class ScanCommandGD : public CommandGD<13> {
public:
    ScanCommandGD() : CommandGD("GD0000000000\n") { }
					
    void setStartValue(int start){
		insertValue(start, 4, 2);
	}
	
	void setEndValue(int end){
		insertValue(end, 4, 6);
	}

};



template<int L>
std::ostream& operator<<(std::ostream& os, const CommandGD<L>& right) {
    for(int i = 0; i < L; i++) {
        os << right[i];
    }
    return os;
};



template<int L>
int& operator<<(int &fd, const CommandGD<L>& right) {

	write(fd, right.cmd.data(), L);
    return fd;
};




#endif /* __CH_NTB_OMNIMOBOT_COMMANDGD_HPP */