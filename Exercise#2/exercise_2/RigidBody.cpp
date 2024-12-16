#include "RigidBody.h"

Mat4 RigidBody::getObj2World() const {
	const auto& p = m_Position;
	const auto& s = m_Size;
	const auto scale_mat = Mat4(
		s.x, 0, 0, 0,
		0, s.y, 0, 0,
		0, 0, s.z, 0,
		0, 0, 0, 1
	);

	const auto translate_mat = Mat4(
		1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		p.x, p.y, p.z, 1
	);

	return scale_mat * m_Orientation.getRotMat() * translate_mat;
}

void RigidBody::applyImpulse(const Vec3& impulse, const Vec3& collisionPoint,  const Vec3& n, const int32_t direction) {
	const auto Jn = impulse * n;
	m_Linear_velocity += direction * Jn / m_Mass;
	m_Momentum += direction * cross(collisionPoint, Jn);
}

std::array<Vec3, 8> RigidBody::getCornersPositions() const {
	std::array<Vec3, 8> corners = {
		Vec3(1, 1, 1), Vec3(1, 1, -1), Vec3(1, -1, 1), Vec3(1, -1, -1), Vec3(-1, 1, 1), Vec3(-1, 1, -1),
		Vec3(-1, -1, 1), Vec3(-1, -1, -1)
	};

	const auto identity = Mat4(
		1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1
	);

	for (auto& point : corners) {
		point = m_Position + m_Size * 0.5 * (identity * point);
		point = m_Orientation.getRotMat() * point;
	}

	return corners;
}

std::vector<Vec3> RigidBody::checkGroundContact() const {
	const auto obj = getObj2World().toDirectXMatrix();
	const auto corners_world = getCorners(obj);
	std::vector<Vec3> contact_points;
	for (const auto& corner : corners_world) {
		const auto x = DirectX::XMVectorGetX(corner);
		const auto y = DirectX::XMVectorGetY(corner);
		const auto z = DirectX::XMVectorGetZ(corner);
		if (y <= -1.0f) { contact_points.emplace_back(x, y, z); }
	}
	return contact_points;
}

void RigidBody::applyForce(const Vec3& location, const Vec3& force) {
	m_Forces.emplace_back(location - m_Position, force);
}

Vec3 RigidBody::getVelocityAtCollision(const Vec3& collisionPoint) const { return m_Linear_velocity + cross(m_Angular_velocity, collisionPoint); }

void RigidBody::updateLinearVelocity(const double time_step) {
	Vec3 force{};
	for (auto& f : m_Forces) { force += f.m_Size; }

	// prevent the bodies from flying away unnoticeably
	const auto new_position = m_Position + time_step * m_Linear_velocity;
	if (new_position.x > -3.0 && new_position.x < 3.0 && new_position.y > -3.0 && new_position.y < 3.0 && new_position.z
		> -3.0 && new_position.z < 3.0) { m_Position = new_position; }

	m_Linear_velocity += time_step * force / static_cast<double>(m_Mass);
}

void RigidBody::updateAngularVelocity(const double time_step) {
	Vec3 torque{};
	for (auto& f : m_Forces) { torque += cross(f.m_Location, f.m_Size); }
	const auto& w = m_Angular_velocity;
	auto quaternion = Quat(w.x, w.y, w.z, 0);
	m_Orientation += time_step / 2.0 * quaternion * m_Orientation;
	m_Orientation /= (m_Orientation.norm() + 0.00001);
	m_Momentum += time_step * torque;

	const auto rot_mat = m_Orientation.getRotMat();
	auto transpose = m_Orientation.getRotMat();
	transpose.transpose();

	m_Inertial_inv = rot_mat * m_Inertial_0_inv * transpose;
	m_Angular_velocity = m_Inertial_inv * m_Momentum;
}
