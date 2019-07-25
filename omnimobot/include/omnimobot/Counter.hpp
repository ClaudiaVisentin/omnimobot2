#ifndef __CH_NTB_OMNIMOBOT_COUNTER_HPP
#define __CH_NTB_OMNIMOBOT_COUNTER_HPP

namespace omnimobot
{
	class Counter {
	public:
		 Counter(int number): 
		 value(number),
		 ctr(number),
		 countEnd(false){			 
		 }
				
		virtual void reset(){
			ctr = value;
			countEnd = false;
		}
		
		virtual void count(){
			ctr = ctr - 1;
			
			if(ctr <= 0){
				countEnd = true;
			}
			else{
				countEnd = false;
			}
		}
		
		virtual bool isCountEnd(){
			return countEnd;
		}
		
	private:
		int value;
		int ctr;
		bool countEnd;
	};
}

#endif /* __CH_NTB_OMNIMOBOT_COUNTER_HPP */