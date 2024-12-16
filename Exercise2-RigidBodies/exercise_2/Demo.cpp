#include "Demo.h"

void Demo::oneStepTest(RigidBodySystem& system) {
	system.m_Gravity_enabled = false;
	system.addRigidBody(Vec3(0.0, 0.0, 0.0), Vec3(1.0, 0.6, 0.5), 2.0);
	system.setOrientationOf(0, Quat(Vec3(0.0, 0.0, 1.0), static_cast<double>((M_PI)) * 0.5));
	system.applyForceOnBody(0, Vec3(0.3, 0.5, 0.25), Vec3(1.0, 1.0, 0.0));
}

void Demo::singleBody(RigidBodySystem& system) {
	system.m_Gravity_enabled = false;
	system.addRigidBody(Vec3(0.0, 0.0, 0.0), Vec3(1.0, 0.6, 0.5), 2.0);
	system.setOrientationOf(0, Quat(Vec3(0.0, 0.0, 1.0), static_cast<double>((M_PI)) * 0.5));
}

void Demo::twoBodies(RigidBodySystem& system) {
	system.m_Gravity_enabled = false;
	system.addRigidBody(Vec3(0.0, 0.0, 0.0), Vec3(0.33, 0.2, 0.1), 1.0);
	system.setOrientationOf(0, Quat(Vec3(0.0, 0.0, 1.0), 0.5 * static_cast<double>(M_PI)));

	system.addRigidBody(Vec3(0.0, 0.6, 0.0), Vec3(0.33, 0.2, 0.1), 1.0);
	system.setOrientationOf(1, Quat(Vec3(0.0, 0.0, 1.0), 0.5 * static_cast<double>(M_PI)));

	system.applyForceOnBody(0, Vec3(0.0, -0.3, 0.0), Vec3(0.0, 500.0, 0.0));
	system.applyForceOnBody(1, Vec3(0.5, 0.7, 1.0), Vec3(0.0, -200.0, 0.0));
}

void Demo::complexSimulation(RigidBodySystem& system) {
	system.m_Gravity_enabled = true;
	system.addRigidBody(Vec3(0.3, -0.5, 0.0), Vec3(0.2, 0.2, 0.2), 1.0);
	system.setOrientationOf(0, Quat(Vec3(0.0, 0.0, 1.0), 0.5 * static_cast<double>(M_PI)));

	system.addRigidBody(Vec3(0.0, -0.5, 0.0), Vec3(0.2, 0.2, 0.2), 1.0);
	system.setOrientationOf(1, Quat(Vec3(0.0, 0.0, 1.0), 0.5 * static_cast<double>(M_PI)));

	system.addRigidBody(Vec3(0.4, 0.5, 0.0), Vec3(0.2, 0.2, 0.2), 1.0);
	system.setOrientationOf(2, Quat(Vec3(0.0, 0.0, 1.0), 0.5 * static_cast<double>(M_PI)));

	system.addRigidBody(Vec3(-0.2, 0.75, 0.0), Vec3(0.2, 0.2, 0.2), 1.0);
	system.setOrientationOf(3, Quat(Vec3(0.0, 0.0, 1.0), 0.5 * static_cast<double>(M_PI)));

	system.applyForceOnBody(0, Vec3(0.0, -0.1, 0.1), Vec3(0.0, 200.0, 0.0));
	system.applyForceOnBody(1, Vec3(0.1, -0.1, 0.0), Vec3(0.0, 200.0, 0.0));
	system.applyForceOnBody(2, Vec3(0.3, -0.1, 0.0), Vec3(100.0, -200.0, 0.0));
	system.applyForceOnBody(3, Vec3(0.0, -0.1, 0.1), Vec3(0.0, 200.0, 0.0));
}
