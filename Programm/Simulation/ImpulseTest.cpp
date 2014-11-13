#include "ImpulseTest.h"
#include "Cube.h"

using namespace Eigen;
ImpulseTest::ImpulseTest()
{
}


ImpulseTest::~ImpulseTest()
{
}

Cube * cube;
bool done = false;
 void ImpulseTest::initializeScene()
{
	 done = false;
	 cube = &Cube::create();

}

 
 void ImpulseTest::update(Real currentTime)
{
	
	 if (!done && currentTime > 5)
	 {
		 cube->addRasImpuls(Vector3d(1.0,1.1,0), Vector3d(-0.5,0.5,0));
		 done = true;
	 }

}
