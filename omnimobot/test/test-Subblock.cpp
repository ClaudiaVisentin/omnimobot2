#include <iostream>

#include <eeros/math/Matrix.hpp>
#include <eeros/control/Input.hpp>
#include <eeros/control/Block.hpp>
#include <eeros/control/Gain.hpp>

namespace
{
	using namespace eeros::control;
	using namespace eeros::math;
	
	class TestBlock : public Block
	{
	public:
		TestBlock() : gain(0.5), input1(gain.getIn())
		{
			
		}
		
		virtual void run() override
		{
			using namespace std;
			
			cout << "TestBlock::run()" << endl;
		}
		
		
		Input<Vector<3>> &getIn()
		{
			return gain.getIn();
		}
		
		
	private:
		Gain<Vector<3>> gain;
	
	public:
		Input<Vector<3>> &input1; // referenzen müssen am schluss kommen
	};

}






// #include <eeros/hal/FlinkDigIn.hpp>


main()
{
	using namespace std;
	
	cout << "eeros subblock test started" << endl;
	
	

	
	cout << "eeros subblock test finished" << endl;
}