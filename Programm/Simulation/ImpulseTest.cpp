#include "ImpulseTest.h"
#include "SimulationManager.h"

#include "Cube.h"
#include "Gravity.h"
#include "SphericalJoint.h"
#include "Spring.h"
#include "Sphere.h"
#include "SimulationManager.h"

using namespace Eigen;
ImpulseTest::ImpulseTest()
{
	name = "Impulse Test";
}


ImpulseTest::~ImpulseTest()
{
}


 void ImpulseTest::initializeScene()
{
	 SimulationManager::getInstance()->getSimulation().setCollisionCheck(false);
	 next = false;
	 timer = 0;
	 
	 Real y = 0;

	 cube01 = &Cube::create();
	 cube01->setPosition(Vector3d(-2, 2.4+y, 0));
	 cube02 = &Cube::create();
	 cube02->setPosition(Vector3d(-2, 1.2+y, 0));
	 cube03 = &Cube::create();
	 cube03->setPosition(Vector3d(-2, y, 0));
	 
	 cube04 = &Cube::create();
	 cube04->setPosition(Vector3d(-2, y-1.2, 0));
	 cube05 = &Cube::create();
	 cube05->setPosition(Vector3d(-2, y-2.4, 0));
	 
	 
	 
	 // Pendular to make it interisting
	 /*
	 Sphere &pendular = Sphere::create(0.2);
	 pendular.setPosition(Vector3d(1, 4, 3));

	 Spring & spring13 = Spring::create();
	 spring13.setSuspensionPoints(Vector3d(0.0, 0, 0), &cube03, Vector3d(0, 0, 0), &pendular);
	 */
	 
}

 
 void ImpulseTest::update(Real currentTime)
{
	 int timeFactor = 5;
	 Real impulseStrength = 0.1;

	 // set Timer
	 if (timer < (int)(currentTime / timeFactor))
	 {
		 timer++;
		 next = true;
	 }
		 
	 if (next)
	 {
		 switch (timer)
		 {
		 case 1:
			 cube01->addRasImpuls(Vector3d(impulseStrength, 0, 0), Vector3d(0, 0, 0));
			 break;
		 case 2:
			 cube02->addRasImpuls(Vector3d(impulseStrength, 0, 0), Vector3d(-1, 1, 0));
			 break;
		 case 3:
			 cube03->addRasImpuls(Vector3d(impulseStrength, 0, 0), Vector3d(-1, 1, 0));
			 cube03->addRasImpuls(Vector3d(impulseStrength, 0, 0), Vector3d(-1, -1, 0));
			 break;
		 case 4:
			 cube04->addRasImpuls(Vector3d(-impulseStrength, impulseStrength, -impulseStrength), Vector3d(-0.5, -0.5, -0.5));
			 cube04->addRasImpuls(Vector3d(impulseStrength, -impulseStrength, impulseStrength), Vector3d(0.5, 0.5, 0.5));
			 break;
		 case 5:
			 cube05->addRasImpuls(Vector3d(impulseStrength, 0, 0), Vector3d(-1, 1, 0));
			 break;
		 case 6:
			 cube05->addRasImpuls(-2*Vector3d(impulseStrength, 0, 0), Vector3d(1, -1, 0));
			 break;
		 case 10:
			// reset();
			 break;
		 }

		 next = false;
	 }
	 
	 

	 
}

 void ImpulseTest::reset()
 {
	 //SimulationManager::getInstance()->getObjectManager().resetObjectManager();
	 //initializeScene();
	 
 }