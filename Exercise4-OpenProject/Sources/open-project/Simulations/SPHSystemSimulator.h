#ifndef SPH_SIMULATOR
#define SPH_SIMULATOR
#include "Simulator.h"
#include "vectorbase.h"
#include <vector>
#include <queue>

const static Vec3 G(0.f, -9.82f, 0); // external (gravitational) forces
const static float REST_DENS = 998.29f; // rest density
const static float GAS_CONST = 3.f; // const for equation of state
const static float SUPPORT_RADIUS = 0.0457f; // kernel radius
const static float SURFACE_TENSION = 0.0728f;
const static float THRESHOLD = 7.065f;
const static float MASS = 0.02; // assume all particles have the same mass
const static float VISC = 3.5f; // viscosity constant

// simulation parameters
const static float RESTITUTION = 0.5f;

// rendering projection parameters
const static float BOX_SIZE = 0.4;
const static float BOX_SIZE_HALF = BOX_SIZE / 2.;
const static float fluidVolume = 1000 * MASS / REST_DENS;
const static float particleDiameter = powf(fluidVolume, 1.0f / 3.0f) / 10;
const static float particleRadius = particleDiameter / 2;

const auto cell_size = 0.015;
const static auto grid_size = int(BOX_SIZE / cell_size);

// for drawing
const static auto sphereRadius = powf((3 * MASS) / (4 * M_PI * REST_DENS), 1.0f / 3.0f);

class Particle {
public:
	Particle(Vec3 pos);

	Vec3 position, velocity, force;
	Real density, pressure;
	std::queue<Real> old_pressures;
	Real sum_pressure;
};

class SPHSystemSimulator final : public Simulator {
public:
	SPHSystemSimulator();

	const char* getTestCasesStr() override;
	void reset() override;
	void initUI(DrawingUtilitiesClass* DUC) override;
	Real colorCoefficient(const double pressure) const;
	double colorCoefficientDensity(double density) const;
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext) override;
	static Mat4 getObj2World(const Vec3& position, const Vec3& size);
	void externalForcesCalculations(float timeElapsed) override;
	void simulateTimestep(float timeStep) override;
	void notifyCaseChanged(int testCase) override;
	void onClick(int x, int y) override;
	void onMouse(int x, int y) override;

	std::vector<Particle> m_Particles;
	std::vector<int> obstacle_cell;
	Real max_pressure;
	Real min_pressure;
	bool pressure_color;
	bool m_Rain;
	int m_Rain_coefficient;
	Real max_density;
	Real min_density;
	std::vector<bool> city_colors;
	bool city_colors_enabled;
	bool surface_tension;
	bool draw_box;

	void computeDensityPressure();
	void computeForces();
	void integrate(float time_step);
	void resolveCollision(float time_step);
	void fillOccupancyGrid(Vec3 pos, Vec3 size);
	int posToGridCell(Vec3 pos) const;
	static Vec3 posFromGridCell(int index);

	bool detectCollision(const Particle& particle, Vec3& contactPoint, Vec3& unitSurfaceNormal) const;

	static float defaultKernel(Vec3 dist);
	static Vec3 defaultKernel_gradient(Vec3 dist);
	static float defaultKernel_laplacian(Vec3 dist);
	static Vec3 pressureKernel_gradient(Vec3 dist);
	static float viscosityKernel_laplacian(Vec3 dist);
	std::vector<Vec3> occupied;

	int32_t m_Test_case;
private:
	static std::pair<Vec3, Vec3> wallContact(const Particle& p, const std::tuple<uint8_t, uint8_t, uint8_t>& axes);
	std::mt19937_64 eng;
};

#endif
