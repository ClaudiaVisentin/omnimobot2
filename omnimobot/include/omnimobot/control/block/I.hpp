#ifndef __CH_NTB_OMNIMOBOT_CONTROL_BLOCK_I_HPP
#define __CH_NTB_OMNIMOBOT_CONTROL_BLOCK_I_HPP

#include <eeros/control/Block1i1o.hpp>
#include <eeros/core/System.hpp>


namespace omnimobot {

	template < typename T = double >
		class I: public eeros::control::Block1i1o<T> {
			
		public:
			I() : first(true) { }
			
			virtual void run() {
				if(first) {  // first run, no previous value available -> set output to zero
					this->out.getSignal().clear();
					this->out.getSignal().setTimestamp(this->in.getSignal().getTimestamp());
					this->prev = this->out.getSignal(); 
					first = false;
				}
				else {
					double tin = this->in.getSignal().getTimestamp() / 1000000000.0;
					double tprev = this->prev.getTimestamp() / 1000000000.0;
					double dt = (tin - tprev);
					T valin = this->in.getSignal().getValue();
					T valprev = this->prev.getValue();
					
					if(enabled){
						// Calculate output
						output = valprev + valin * dt;
							
					}
					else{
						output = valprev;
						
					}
					this->out.getSignal().setValue(output);
					this->out.getSignal().setTimestamp(this->in.getSignal().getTimestamp());
// 					this->out.getSignal().setTimestamp((this->in.getSignal().getTimestamp() + this->prev.getTimestamp()) / 2);
					this->prev = this->out.getSignal();
				}
			}
			
			virtual void enable() {
				this->enabled = true;
			}
			virtual void disable() {
				this->enabled = false;
			}
			virtual void setInitCondition(T pos) {
				this->prev.setValue(pos);
				this->prev.setTimestamp(this->out.getSignal().getTimestamp());
			}

			eeros::control::Signal<T> prev;
			
		protected:
			bool first;
			bool enabled = false;
			T output; 

		};
	};
#endif /* __CH_NTB_OMNIMOBOT_CONTROL_BLOCK_I_HPP */
