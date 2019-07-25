#include <Math.hpp>

using namespace omnimobot::math;

double Math::fabs(double value)
{
	if (value < 0)
		return -value;
	else
		return value;
}

int Math::power(int base, int exp)
{
    int result = 1;
    while(exp) {
        result *= base;
        exp--;
    }
    return result;
}
