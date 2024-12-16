#ifndef RIGID_BODY_SYSTEM_H
#define RIGID_BODY_SYSTEM_H
#include <cstdint>
#include <vector>
#include "RigidBody.h"
#include "util/vectorbase.h"
#include "util/quaternion.h"
#include "collisionDetect.h"

using GamePhysics::Vec3;
using GamePhysics::Quat;

class RigidBodySystem {
public:
	RigidBodySystem(): m_Gravity_enabled(false) {
	}

	void applyForceOnBody(size_t i, const Vec3& loc, const Vec3& force);
	void addRigidBody(const Vec3& position, const Vec3& size, uint32_t mass);
	void setOrientationOf(size_t i, const Quat& orientation);
	void setVelocityOf(size_t i, const Vec3& velocity);
	void simulateTimestep(double time_step);

	std::vector<RigidBody> m_Rigid_bodies;
	bool m_Gravity_enabled;
private:
	void applyGravity();
	void resolveBodyCollisions();
	void resolveGroundCollisions();

	static void resolveCollision(RigidBody& a, RigidBody& b, const CollisionInfo& collision);
};
#endif
