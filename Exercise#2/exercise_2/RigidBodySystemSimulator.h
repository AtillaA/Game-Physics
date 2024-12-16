#ifndef RIGIDBODYSYSTEMSIMULATOR_h
#define RIGIDBODYSYSTEMSIMULATOR_h
#include "RigidBodySystem.h"
#include "Simulator.h"

constexpr uint32_t testcase_used_to_run_test = 2;

class RigidBodySystemSimulator final : public Simulator
{
public:
	// Constructor
	RigidBodySystemSimulator();

	// Functions
	const char* getTestCasesStr() override;
	void initUI(DrawingUtilitiesClass* duc) override;
	void reset() override;
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext) override;
	void notifyCaseChanged(uint32_t test_case) override;
	void externalForcesCalculations(double time_elapsed) override;
	void simulateTimestep(double time_step) override;
	void onClick(int32_t x, int32_t y) override;
	void onMouse(int32_t x, int32_t y) override;

	// ExtraFunctions
	size_t getNumberOfRigidBodies() const;
	Vec3 getPositionOfRigidBody(size_t i) const;
	Vec3 getLinearVelocityOfRigidBody(size_t i) const;
	Vec3 getAngularVelocityOfRigidBody(size_t i) const;

	void applyForceOnBody(size_t i, const Vec3& loc, const Vec3& force);
	void addRigidBody(const Vec3& position, const Vec3& size, uint32_t mass);
	void setOrientationOf(size_t i, const Quat& orientation);
	void setVelocityOf(size_t i, const Vec3& velocity);
private:
	// UI Attributes
	Point2D m_Mouse;
	Point2D m_Track_mouse;
	Point2D m_Old_track_mouse;

	uint32_t m_Test_case;

	RigidBodySystem m_Rigid_body_system;
};
#endif
