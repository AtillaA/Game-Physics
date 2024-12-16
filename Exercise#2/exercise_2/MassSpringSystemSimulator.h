#ifndef MASSSPRINGSYSTEMSIMULATOR_h
#define MASSSPRINGSYSTEMSIMULATOR_h
#include "Simulator.h"

// Do Not Change
#define EULER 0
#define LEAPFROG 1
#define MIDPOINT 2
// Do Not Change

struct PoseVel3D {
	Vec3 pose;
	Vec3 vel;
	int is_fixed;
};

struct Spring {
	int mpoint1;
	int mpoint2;
	float initial_length;
};

using std::vector;

class MassSpringSystemSimulator : public Simulator {
public:
	// Construtors
	MassSpringSystemSimulator();

	// UI Functions
	const char* getTestCasesStr() override;
	// const char* getIntegratorStr();
	void initUI(DrawingUtilitiesClass* DUC) override;
	void reset() override;
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext) override;
	void notifyCaseChanged(uint32_t testCase) override;
	void externalForcesCalculations(double timeElapsed) override;
	void simulateTimestep(double timeStep) override;
	void onClick(int x, int y) override;
	void onMouse(int x, int y) override;

	// Specific Functions
	void setMass(float mass);
	void setStiffness(float stiffness);
	void setDampingFactor(float damping);
	int addMassPoint(Vec3 position, Vec3 velocity, bool is_fixed);
	void addSpring(int mass_point1, int mass_point2, float initial_length);
	int getNumberOfMassPoints() const;
	int getNumberOfSprings() const;
	Vec3 getPositionOfMassPoint(int index);
	Vec3 getVelocityOfMassPoint(int index);
	void applyExternalForce(Vec3 force);

	// Do Not Change
	void setIntegrator(int integrator) { m_iIntegrator = integrator; }

	vector<Vec3> compute_accelerations(vector<PoseVel3D> input_points);

private:
	// Data Attributes
	float m_fMass;
	float m_fStiffness;
	float m_fDamping;
	int m_iIntegrator;

	// UI Attributes
	Vec3 m_externalForce;
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;

	// MassSpringSystem Attributes
	vector<PoseVel3D> points;
	vector<Spring> springs;
	vector<Vec3> forces;

	// Other properties
	bool gravity = false;
	const double mp_size = 0.1;
};
#endif
