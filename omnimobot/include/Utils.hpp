#ifndef __CH_NTB_OMNIMOBOT_UTILS_HPP
#define __CH_NTB_OMNIMOBOT_UTILS_HPP

class Utils {
	public:
		/** compare
		 * @param tolerance relative to ref
		 */
		static bool compareApprox(double ref, double val, double tolerance) {
			if (ref == val) {
				return true;
			}
			else {
				double diff = (ref - val);
				if (diff < 0) diff = -diff;
				if (ref < 0) ref = -ref;
				return (diff < tolerance*ref);
			}
		}
		
		static double abs(double a) {
			if(a >= 0) {
				return a;
			}
			else {
				return -a;
			}
		}
};
		
#endif // __CH_NTB_OMNIMOBOT_UTILS_HPP
