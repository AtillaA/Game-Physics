#ifndef DEMO_H
#define DEMO_H
#include "RigidBodySystem.h"

class Demo {
public:
	enum Type {
		OneStepTest = 0,
		SingleBody = 1,
		TwoBodiesCollision = 2,
		ComplexSimulation = 3,
	};

	static void oneStepTest(RigidBodySystem& system);
	static void singleBody(RigidBodySystem& system);
	static void twoBodies(RigidBodySystem& system);
	static void complexSimulation(RigidBodySystem& system);
};
#endif
