#include "ImpulseTest.h"
#include "Cube.h"
#include "Gravity.h"
#include "SphericalJoint.h"

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

	 Cube & cube01 = Cube::create();
	 cube01.setMass(0);
	 Cube & cube02 = Cube::create();
	 cube02.setPosition(Vector3d(1, 0, 0));
	 SphericalJoint &joint01 = SphericalJoint::create(&cube01, &cube02, Vector3d(0.5, -0.5, 0));
	 Gravity::create();
}

 
 void ImpulseTest::update(Real currentTime)
{
	 /*
	 if (!done && currentTime > 5)
	 {
		 cube->addRasImpuls(Vector3d(1.0,1.1,0), Vector3d(-0.5,0.5,0));
		 done = true;
	 }
	 */
}
