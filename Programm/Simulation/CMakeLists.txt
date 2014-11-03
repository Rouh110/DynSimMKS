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
	  IForce.h
	  ObjectManager.cpp
	  ObjectManager.h
	  RigidBody.cpp
	  RigidBody.h
	  Simulation.cpp
	  Simulation.h
	  SimulationManager.cpp
	  SimualtionManager.h
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
