#include "RigidBodySystem.h"

void RigidBodySystem::addRigidBody(const Vec3& position, const Vec3& size, const uint32_t mass) {
	m_Rigid_bodies.emplace_back(position, size, mass);
}

void RigidBodySystem::applyForceOnBody(const size_t i, const Vec3& loc, const Vec3& force) {
	if (i < m_Rigid_bodies.size())
		m_Rigid_bodies[i].applyForce(loc, force);
}

void RigidBodySystem::setOrientationOf(const size_t i, const Quat& orientation) {
	if (i < m_Rigid_bodies.size())
		m_Rigid_bodies[i].m_Orientation = orientation;
}

void RigidBodySystem::setVelocityOf(const size_t i, const Vec3& velocity) {
	if (i < m_Rigid_bodies.size())
		m_Rigid_bodies[i].m_Linear_velocity = velocity;
}

void RigidBodySystem::simulateTimestep(const double time_step) {
	if (m_Gravity_enabled && !m_Rigid_bodies.empty()) { applyGravity(); }

	for (auto& b : m_Rigid_bodies) {
		b.updateLinearVelocity(time_step);
		b.updateAngularVelocity(time_step);
		b.m_Forces.clear();
	}

	resolveBodyCollisions();
	resolveGroundCollisions();
}

void RigidBodySystem::applyGravity() {
	for (auto& b : m_Rigid_bodies) { 
		//if (b.m_Position.y >= -1.0f) {
			b.applyForce(b.m_Position, Vec3(0, -10., 0) * b.m_Mass);
		//}
	}
}

void RigidBodySystem::resolveBodyCollisions() {
	for (size_t i = 0; i < m_Rigid_bodies.size(); ++i) {
		auto& a = m_Rigid_bodies[i];
		for (auto j = i+1; j < m_Rigid_bodies.size(); ++j) {
			auto& b = m_Rigid_bodies[j];

			auto obj_a = a.getObj2World();
			auto obj_b = b.getObj2World();

			const auto collision = checkCollisionSAT(obj_a, obj_b);
			if (collision.isValid) { resolveCollision(a, b, collision); }
		}
	}
}

void RigidBodySystem::resolveGroundCollisions() {
	for (auto& b : m_Rigid_bodies) {
		auto contact_points = b.checkGroundContact();
		if (contact_points.empty()) continue;

		const auto x_a = Vec3(0., -1. - b.m_Position.y, 0.);

		const auto v_rel = b.getVelocityAtCollision(x_a);

		auto n = Vec3(0, 1., 0);
		/*for (const auto& point : contact_points) { n += point; }
		n /= contact_points.size();
		n = b.m_Position - n;
		n /= sqrt(pow(n.x, 2) + pow(n.y, 2) + pow(n.z, 2)) + DBL_EPSILON;*/

		const auto bounciness = 0.5;
		const auto top = -(1.0 + bounciness) * v_rel * n;
		const auto cross_b = cross(b.m_Inertial_inv * cross(x_a, n), x_a);
		const auto bottom = 1. / static_cast<double>(b.m_Mass) + cross_b * n;
		const auto impulse = top / bottom;
		b.applyImpulse(impulse, x_a, n, 1);
	}
}

void RigidBodySystem::resolveCollision(RigidBody& a, RigidBody& b, const CollisionInfo& collision) {
	// 0. calculate collision position
	const auto x_a = collision.collisionPointWorld - a.m_Position;
	const auto x_b = collision.collisionPointWorld - b.m_Position;
	
	// 1. calculate velocities at collision point for each rigid body
	const auto v_a = a.getVelocityAtCollision(x_a);
	const auto v_b = b.getVelocityAtCollision(x_b);

	// 2. calculate relative velocity
	auto v_rel = v_a - v_b;

	// 3. calculate impulse
	const auto bounciness = 0.5;
	auto& n = collision.normalWorld;
	const auto top = -(1.0 + bounciness) * v_rel * n;
	const auto cross_a = cross(a.m_Inertial_inv * cross(x_a, n), x_a);
	const auto cross_b = cross(b.m_Inertial_inv * cross(x_b, n), x_b);
	const auto bottom = (1. / static_cast<double>(a.m_Mass)) + (1. / static_cast<double>(b.m_Mass)) + (cross_a +
		cross_b) * n;
	const auto impulse = top / bottom;

	// 4. apply impulse on both bodies but in opposite directions
	a.applyImpulse(impulse, x_a, n, 1);
	b.applyImpulse(impulse, x_b, n, -1);
}
