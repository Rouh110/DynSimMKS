SET(SIMULATION_LINK_LIBRARIES freeglut opengl32.lib glu32.lib ${ANTTWEAKBAR_LIBS})
SET (SIMULATION_DEPENDENCIES freeglut)

# PROJECT(Simulation)
ADD_EXECUTABLE(Simulation
	  main.cpp
	  Particle.cpp
	  Particle.h  
	  SimMath.cpp
	  SimMath.h
	  TimeManager.cpp
	  TimeManager.h
	  Cube.cpp
	  Cube.h
	  IForce.cpp
	  IForce.h
	  ObjectManager.cpp
	  ObjectManager.h
	  RigidBody.cpp
	  RigidBody.h
	  Simulation.cpp
	  Simulation.h
	  SimulationManager.cpp
	  SimulationManager.h
	  Sphere.cpp
	  Sphere.h
	  Spring.cpp
	  Spring.h
	  GlobalForce.cpp
	  GlobalForce.h
	  Gravity.cpp
	  Gravity.h
	  IScene.cpp
	  IScene.h
	  TestScene01.cpp
	  TestScene01.h
	  TestScene02.cpp
	  TestScene02.h
	  TestScene03.cpp
	  TestScene03.h
	  IJoint.cpp
	  IJoint.h
	  SphericalJoint.cpp
	  SphericalJoint.h
	  ImpulseTest.cpp
	  ImpulseTest.h
	  SceneMobile.cpp
	  SceneMobile.h
	  ScenePuppet.cpp
	  ScenePuppet.h
	  BoundingVolume.cpp
	  BoundingVolume.h
	  BoundingVolumeTree.cpp
	  BoundingVolumeTree.h
	  BoundingVolumeTreeNode.cpp
	  BoundingVolumeTreeNode.h
	  SceneBoundingVolume.h
	  SceneBoundingVolumeTwo.h
	  SceneBoundingVolume.cpp
	  SceneBoundingVolumeTwo.cpp
	  SceneCollisionTest01.cpp
	  SceneCollisionTest01.h
	  SceneCollisionTest02.cpp
	  SceneCollisionTest02.h
	  SceneStressTest01.cpp
	  SceneStressTest01.h
	  SceneStressTest02.cpp
	  SceneStressTest02.h
	  ${VIS_FILES}                    
	  ${PROJECT_PATH}/Common/Config.h			  
	  ${PROJECT_PATH}/Common/StringTools.h			  
	  ${PROJECT_PATH}/Common/StringTools.cpp
	  ${PROJECT_PATH}/Common/timing.h			  	  
	  ${PROJECT_PATH}/Common/timing.cpp
	  CMakeLists.txt
)

find_package( Eigen3 REQUIRED )
include_directories( ${EIGEN3_INCLUDE_DIR} )

ADD_DEPENDENCIES(Simulation ${SIMULATION_DEPENDENCIES})
TARGET_LINK_LIBRARIES(Simulation ${SIMULATION_LINK_LIBRARIES})
VIS_SOURCE_GROUPS()
COPY_ANTTWEAKBAR_DLL(Simulation)
