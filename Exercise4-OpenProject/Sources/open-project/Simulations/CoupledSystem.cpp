#include "CoupledSystem.h"

CoupledSimulator::CoupledSimulator() {
}

const char* CoupledSimulator::getTestCasesStr() { return m_sph_sim.getTestCasesStr(); }

void CoupledSimulator::initUI(DrawingUtilitiesClass* duc) {
	m_sph_sim.initUI(duc);
	m_rb_sim.initUI(duc);
}

void CoupledSimulator::reset() {
	m_sph_sim.reset();
	m_rb_sim.reset();
}

void CoupledSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext) {
	m_sph_sim.drawFrame(pd3dImmediateContext);
	m_rb_sim.drawFrame(pd3dImmediateContext);
}

void CoupledSimulator::notifyCaseChanged(int test_case) {
	m_sph_sim.notifyCaseChanged(test_case);
	m_rb_sim.m_Rigid_body_system.m_Rigid_bodies.clear();
	m_sph_sim.occupied.clear();
	m_sph_sim.city_colors.clear();

	switch (test_case) {
	case 0: {
		// Simplicity of Water
		break;
	}
	case 1: {
		// Tsunami
		const Vec3 rb_size = Vec3(0.05, BOX_SIZE, 0.10);
		const Vec3 rb_pos = Vec3(0.1, -BOX_SIZE_HALF + rb_size.y / 2, -0.03);

		m_sph_sim.fillOccupancyGrid(rb_pos, rb_size / 2);
		break;
	}
	case 2: {
		// Rainy day at the sea
		break;
	}
	case 3: {
		// Rain and the city
		Vec3 rb_size(0.05, 0.1, 0.10);
		Vec3 rb_pos(0.1, -BOX_SIZE_HALF + rb_size.y / 2 - cell_size, -0.03);

		m_sph_sim.fillOccupancyGrid(rb_pos, rb_size / 2);

		rb_size = Vec3(0.05, 0.15, 0.05);
		rb_pos = Vec3(-0.05, -BOX_SIZE_HALF + rb_size.y / 2 - cell_size, 0.02);

		m_sph_sim.fillOccupancyGrid(rb_pos, rb_size / 2);
		break;
	}
	case 4: {
		const Vec3 rb_size = Vec3(cell_size * 3, 0.05, BOX_SIZE + cell_size * 2);
		const Vec3 rb_pos = Vec3(0.05, -BOX_SIZE_HALF + cell_size * 1.2, -cell_size);

		m_sph_sim.fillOccupancyGrid(rb_pos, rb_size / 2);
		// Overflowing pool
		break;
	}
	default: break;
	}


	/*for (int32_t i = 0; i < m_rb_cnt; ++i)
		m_rb_sim.addRigidBody(Vec3() + Vec3(0.03, 0.03, 0.03) * i, Vec3(0.03, 0.03, 0.03), 2);*/
	//m_rb_sim.notifyCaseChanged(test_case);
}

void CoupledSimulator::externalForcesCalculations(float time_elapsed) {
	m_sph_sim.externalForcesCalculations(time_elapsed);
	m_rb_sim.externalForcesCalculations(time_elapsed);
}

void CoupledSimulator::simulateTimestep(float time_step) {
	m_sph_sim.simulateTimestep(time_step);
	// m_rb_sim.simulateTimestep(time_step);
}

void CoupledSimulator::onClick(int32_t x, int32_t y) { m_rb_sim.onClick(x, y); }

void CoupledSimulator::onMouse(int32_t x, int32_t y) { m_rb_sim.onMouse(x, y); }
