#ifndef RIGIDBODYSYSTEMSIMULATOR_h
#define RIGIDBODYSYSTEMSIMULATOR_h
#include "RigidBodySystem.h"
#include "Simulator.h"
#include <stdint.h>

constexpr uint32_t testcase_used_to_run_test = 2;

class RigidBodySystemSimulator final : public Simulator {
public:
	// Constructor
	RigidBodySystemSimulator();

	// Functions
	const char* getTestCasesStr() override;
	void initUI(DrawingUtilitiesClass* duc) override;
	void reset() override;
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext) override;
	void notifyCaseChanged(int test_case) override;
	void externalForcesCalculations(float time_elapsed) override;
	void simulateTimestep(float time_step) override;
	void onClick(int32_t x, int32_t y) override;
	void onMouse(int32_t x, int32_t y) override;

	// ExtraFunctions
	uint32_t getNumberOfRigidBodies() const;
	Vec3 getPositionOfRigidBody(uint32_t i) const;
	Vec3 getLinearVelocityOfRigidBody(uint32_t i) const;
	Vec3 getAngularVelocityOfRigidBody(uint32_t i) const;

	void applyForceOnBody(uint32_t i, const Vec3& loc, const Vec3& force);
	void addRigidBody(const Vec3& position, const Vec3& size, uint32_t mass);
	void setOrientationOf(uint32_t i, const Quat& orientation);
	void setVelocityOf(uint32_t i, const Vec3& velocity);
	RigidBodySystem m_Rigid_body_system;
private:
	// UI Attributes
	Point2D m_Mouse;
	Point2D m_Track_mouse;
	Point2D m_Old_track_mouse;

	uint32_t m_Test_case;
};
#endif
