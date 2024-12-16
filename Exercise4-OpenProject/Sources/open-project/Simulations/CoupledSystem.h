#ifndef COUPLED_SIMULATOR_H
#define COUPLED_SIMULATOR_H

#include "Simulator.h"
#include "SPHSystemSimulator.h"
#include "RigidBodySystemSimulator.h"


class CoupledSimulator : public Simulator {
public:
	CoupledSimulator();

	const char* getTestCasesStr() override;
	void initUI(DrawingUtilitiesClass* duc) override;
	void reset() override;
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext) override;
	void notifyCaseChanged(int test_case) override;
	void externalForcesCalculations(float time_elapsed) override;
	void simulateTimestep(float time_step) override;
	void onClick(int32_t x, int32_t y) override;
	void onMouse(int32_t x, int32_t y) override;
	SPHSystemSimulator m_sph_sim;
	RigidBodySystemSimulator m_rb_sim;
	int32_t m_Test_case;
};

#endif
