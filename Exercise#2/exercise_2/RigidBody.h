#ifndef RIGID_BODY_H
#define RIGID_BODY_H
#include <vector>
#include <array>
#include "Force.h"
#include "util/matrixbase.h"
#include "util/vectorbase.h"
#include "util/quaternion.h"
#include "collisionDetect.h"

using GamePhysics::Vec3;
using GamePhysics::Mat4;
using GamePhysics::Quat;
using collisionTools::getCorners;

class RigidBody {
public:
	Vec3 m_Position;
	Vec3 m_Size;
	uint32_t m_Mass;

	Vec3 m_Linear_velocity;
	Vec3 m_Angular_velocity;

	Quat m_Orientation;
	Vec3 m_Momentum;

	Mat4 m_Inertial_0_inv;
	Mat4 m_Inertial_inv;

	std::vector<Force> m_Forces;
	std::vector<std::tuple<Vec3, Vec3, int32_t>> m_Impulses;

	RigidBody(const Vec3& position, const Vec3& size, const uint32_t mass) : m_Position(position),
	                                                                         m_Size(size),
	                                                                         m_Mass(mass),
	                                                                         m_Orientation(Quat(0, 0, 0, 1)) {
		const auto& w = size[0];
		const auto& d = size[1];
		const auto& h = size[2];

		const auto& a = 12. / (mass * (pow(h, 2) + pow(d, 2)));
		const auto& b = 12. / (mass * (pow(w, 2) + pow(d, 2)));
		const auto& c = 12. / (mass * (pow(w, 2) + pow(h, 2)));

		m_Inertial_0_inv = Mat4(
			a, 0, 0, 0,
			0, b, 0, 0,
			0, 0, c, 0,
			0, 0, 0, 1
		);
	}

	Mat4 getObj2World() const;
	Vec3 getVelocityAtCollision(const Vec3& collisionPoint) const;
	void updateLinearVelocity(double time_step);
	void updateAngularVelocity(double time_step);

	void applyImpulse(const Vec3& impulse, const Vec3& collisionPoint, const Vec3& n, int32_t direction);
	std::array<Vec3, 8> getCornersPositions() const;
	std::vector<Vec3> checkGroundContact() const;
	void applyForce(const Vec3& location, const Vec3& force);
};
#endif:
