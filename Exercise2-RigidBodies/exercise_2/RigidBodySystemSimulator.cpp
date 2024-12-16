#include "RigidBodySystemSimulator.h"
#include "Demo.h"

RigidBodySystemSimulator::RigidBodySystemSimulator(): m_Mouse(), m_Track_mouse(), m_Old_track_mouse(), m_Test_case(0) {
}

const char* RigidBodySystemSimulator::getTestCasesStr() {
	return "1:One-step test,2:Single body,3:Two bodies collision,4:Complex simulation";
}

void RigidBodySystemSimulator::initUI(DrawingUtilitiesClass* duc) { this->DUC = duc; }

void RigidBodySystemSimulator::reset() {
	m_Mouse.x = m_Mouse.y = 0;
	m_Track_mouse.x = m_Track_mouse.y = 0;
	m_Old_track_mouse.x = m_Old_track_mouse.y = 0;
}

void RigidBodySystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext) {
	std::mt19937 eng;
	const std::uniform_real_distribution<double> randCol(0.0, 1.0);

	for (const auto& b : m_Rigid_body_system.m_Rigid_bodies) {
		DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, 0.6 * Vec3(randCol(eng), randCol(eng), randCol(eng)));
		const auto obj2_world_matrix = b.getObj2World();
		DUC->drawRigidBody(obj2_world_matrix);
	}
}

void RigidBodySystemSimulator::notifyCaseChanged(const uint32_t test_case) {
	m_Test_case = test_case;

	m_Rigid_body_system.m_Rigid_bodies.clear();

	switch (m_Test_case) {
	case Demo::Type::OneStepTest:
		Demo::oneStepTest(m_Rigid_body_system);
		break;
	case Demo::Type::SingleBody:
		Demo::singleBody(m_Rigid_body_system);
		break;
	case Demo::Type::TwoBodiesCollision:
		Demo::twoBodies(m_Rigid_body_system);
		break;
	case Demo::Type::ComplexSimulation:
		Demo::complexSimulation(m_Rigid_body_system);
		break;
	default:
		break;
	}
}

void RigidBodySystemSimulator::onClick(const int32_t x, const int32_t y) {
	m_Track_mouse.x = x;
	m_Track_mouse.y = y;
}

void RigidBodySystemSimulator::onMouse(const int32_t x, const int32_t y) {
	m_Old_track_mouse.x = x;
	m_Old_track_mouse.y = y;
	m_Track_mouse.x = x;
	m_Track_mouse.y = y;
}

void RigidBodySystemSimulator::externalForcesCalculations(const double time_elapsed) {
	const Point2D mouse_diff{m_Track_mouse.x - m_Old_track_mouse.x, m_Track_mouse.y - m_Old_track_mouse.y};

	if (mouse_diff.x != 0 || mouse_diff.y != 0) {
		auto world_view_inv = Mat4(DUC->g_camera.GetWorldMatrix() * DUC->g_camera.GetViewMatrix());
		world_view_inv = world_view_inv.inverse();

		const auto input_view = Vec3(static_cast<double>(mouse_diff.x), static_cast<double>(-mouse_diff.y), 0);
		const auto location_view = Vec3(static_cast<double>(m_Old_track_mouse.x),
		                                static_cast<double>(-m_Old_track_mouse.y), 0);

		auto input_world = world_view_inv.transformVectorNormal(input_view);
		auto location_world = world_view_inv.transformVectorNormal(location_view);

		input_world *= 0.1;
		location_world *= 0.001;
		applyForceOnBody(0, location_world, input_world);
	}
}

void RigidBodySystemSimulator::simulateTimestep(double time_step) { m_Rigid_body_system.simulateTimestep(time_step); }

size_t RigidBodySystemSimulator::getNumberOfRigidBodies() const { return m_Rigid_body_system.m_Rigid_bodies.size(); }

Vec3 RigidBodySystemSimulator::getPositionOfRigidBody(const size_t i) const {
	return m_Rigid_body_system.m_Rigid_bodies[i].m_Position;
}

Vec3 RigidBodySystemSimulator::getLinearVelocityOfRigidBody(const size_t i) const {
	return m_Rigid_body_system.m_Rigid_bodies[i].m_Linear_velocity;
}

Vec3 RigidBodySystemSimulator::getAngularVelocityOfRigidBody(const size_t i) const {
	return m_Rigid_body_system.m_Rigid_bodies[i].m_Angular_velocity;
}

void RigidBodySystemSimulator::applyForceOnBody(size_t i, const Vec3& loc, const Vec3& force) {
	m_Rigid_body_system.applyForceOnBody(i, loc, force);
}

void RigidBodySystemSimulator::addRigidBody(const Vec3& position, const Vec3& size, const uint32_t mass) {
	m_Rigid_body_system.addRigidBody(position, size, mass);
}

void RigidBodySystemSimulator::setOrientationOf(const size_t i, const Quat& orientation) {
	m_Rigid_body_system.setOrientationOf(i, orientation);
}

void RigidBodySystemSimulator::setVelocityOf(const size_t i, const Vec3& velocity) {
	m_Rigid_body_system.setVelocityOf(i, velocity);
}
