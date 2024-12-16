#ifndef DIFFUSION_SIMULATOR_H
#define DIFFUSION_SIMULATOR_H

#include "Simulator.h"
// #include "vectorbase.h"

//implement your own grid class for saving grid data
class Grid {
public:
	// Constructors
	Grid(const uint32_t width, const uint32_t height, const uint32_t depth = 1)
		: m_Width(width), m_Height(height), m_Depth(depth) {
		m_Values = std::vector<std::vector<std::vector<double>>>(
			m_Width, std::vector<std::vector<double>>(m_Height, std::vector<double>(m_Depth, 0.0)));

		std::mt19937_64 eng(std::random_device{}());
		std::uniform_real_distribution<double> rand_value(-1.0, 1.0);

		for (auto& rectangle : m_Values) {
			for (auto& row : rectangle) {
				for (auto& cell : row) {
					// random  value
					cell = rand_value(eng);
				}
			}
		}
	}

	uint32_t m_Width;
	uint32_t m_Height;
	uint32_t m_Depth;

	std::vector<std::vector<std::vector<double>>> m_Values;
};


class DiffusionSimulator final : public Simulator {
public:
	// Constructors
	DiffusionSimulator();

	// Functions
	const char* getTestCasesStr() override;
	void initUI(DrawingUtilitiesClass* duc) override;
	void reset() override;
	void drawFrame(ID3D11DeviceContext* pd3d_immediate_context) override;
	void notifyCaseChanged(int test_case) override;
	void simulateTimestep(float time_step) override;

	void externalForcesCalculations(float time_elapsed) override {
	}

	void onClick(int x, int y) override;
	void onMouse(int x, int y) override;
	// Specific Functions
	void drawObjects();
	std::shared_ptr<Grid> diffuseTemperatureExplicit(double time_step);
	void diffuseTemperatureImplicit(double time_step);

private:
	// Attributes
	Vec3 m_Vf_movable_object_pos;
	Vec3 m_Vf_movable_object_final_pos;
	Vec3 m_Vf_rotate;
	Point2D m_Mouse;
	Point2D m_Track_mouse;
	Point2D m_Old_track_mouse;

	struct GridSize {
		uint32_t nx;
		uint32_t ny;
		uint32_t nz;
	};

	GridSize m_Grid_size;
	float m_Alpha;

	std::shared_ptr<Grid> m_T; //save results of every time step
};

#endif
