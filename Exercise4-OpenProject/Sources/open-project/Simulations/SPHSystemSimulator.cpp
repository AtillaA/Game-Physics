#include "SPHSystemSimulator.h"

Particle::Particle(Vec3 pos): position(move(pos)), velocity(0, 0, 0), force(0, 0, 0),
                              density(0), pressure(0), sum_pressure(0) {
}

SPHSystemSimulator::SPHSystemSimulator(): max_pressure(0), min_pressure(0), pressure_color(true),
                                          m_Rain(false), m_Rain_coefficient(5), max_density(0), min_density(0),
                                          city_colors_enabled(false), surface_tension(true), draw_box(true),
                                          m_Test_case(0) {
	eng = std::mt19937_64(time(nullptr));
	srand(time(nullptr));
}

const char* SPHSystemSimulator::getTestCasesStr() {
	return "Simplicity of Water,Tsunami,Rainy day at the sea,Rain and the city,Overflowing pool";
}

void SPHSystemSimulator::reset() {
}

void SPHSystemSimulator::initUI(DrawingUtilitiesClass* DUC) {
	this->DUC = DUC;
	TwAddVarRW(DUC->g_pTweakBar, "Color by pressure", TW_TYPE_BOOLCPP, &pressure_color, "");
	TwAddVarRW(DUC->g_pTweakBar, "Rain", TW_TYPE_BOOLCPP, &m_Rain, "");
	TwAddVarRW(DUC->g_pTweakBar, "Rain coefficient", TW_TYPE_INT32, &m_Rain_coefficient, "min=2");
	TwAddVarRW(DUC->g_pTweakBar, "City colors", TW_TYPE_BOOLCPP, &city_colors_enabled, "");
	TwAddVarRW(DUC->g_pTweakBar, "Surface tension", TW_TYPE_BOOLCPP, &surface_tension, "");
	TwAddVarRW(DUC->g_pTweakBar, "Draw box", TW_TYPE_BOOLCPP, &draw_box, "");
}

double SPHSystemSimulator::colorCoefficient(const double pressure) const {
	static const auto coefficient_threshold = 0.75;
	static const auto coefficient_shrink = 5.;
	static const auto thr_top = coefficient_threshold + 0.05;
	static const auto thr_bottom = coefficient_threshold - 0.4;

	auto shrink = [](const double coefficient) { return coefficient / coefficient_shrink; };
	auto grow = [](const double coefficient, const double threshold) {
		return (coefficient - threshold) * ((1 - (1 / coefficient_shrink)) / (1 - threshold)) + 1 /
			coefficient_shrink;
	};

	const auto press_range = max_pressure - min_pressure + FLT_EPSILON;
	auto raw_coefficient = [this, press_range](const double p) { return 1 - ((p - this->min_pressure) / press_range); };

	const auto c_new = raw_coefficient(pressure);

	if (c_new < coefficient_threshold) return shrink(c_new);
	return grow(c_new, coefficient_threshold);
}

void SPHSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext) {
	static const auto dark_red = Vec3(0.5, 0, 0);
	static const auto dark_blue = Vec3(0, 0.13, 0.33);
	static const auto white = Vec3(1, 1, 1);
	static const auto col_range = white - dark_blue;
	static const auto col_range_red = white - dark_red;

	for (const auto& p : m_Particles) {
		auto color = dark_blue;
		if (pressure_color) {
			color =
				col_range * colorCoefficient(p.pressure) + dark_blue;
		}

		DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, color);
		DUC->drawSphere(
			p.position,
			Vec3(static_cast<double>(sphereRadius), static_cast<double>(sphereRadius),
			     static_cast<double>(sphereRadius)));
	}
	uint32_t index = 0;
	for (const auto& pos : occupied) {
		if (city_colors_enabled) {
			auto color = Vec3(0.1, 0.1, 0.1);
			auto lit = city_colors[index];
			if (lit) { color = Vec3(1, 1, 0); }
			if (rand() % (lit ? 1000 : 6000) == 1)
				city_colors[index] = !lit;
			// if (rand() % 6 == 1) { color = Vec3(1, 1, 0); }

			DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, color);
		}
		else { DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, Vec3(0, (pos.y + BOX_SIZE_HALF) / BOX_SIZE, 0.1)); }
		const auto obj2_world_matrix = getObj2World(pos, Vec3(cell_size, cell_size, cell_size));
		DUC->drawRigidBody(obj2_world_matrix);

		index += 1;
	}
	std::vector<std::vector<Vec3>> p1 = {
		{
			Vec3(-1, -1, -1), Vec3(-1, -1, 1), Vec3(-1, 1, 1), Vec3(1, 1, 1), Vec3(1, -1, 1), Vec3(1, -1, -1),
			Vec3(1, 1, -1), Vec3(-1, 1, -1), Vec3(-1, -1, -1), Vec3(1, -1, -1)
		},
		{Vec3(-1, -1, 1), Vec3(1, -1, 1)}, {Vec3(1, 1, -1), Vec3(1, 1, 1)}, {Vec3(-1, 1, -1), Vec3(-1, 1, 1)},
	};
	if (draw_box) {
		for (uint32_t j = 0; j < p1.size(); ++j) {
			DUC->beginLine();
			for (uint32_t i = 1; i < p1[j].size(); ++i) {
				DUC->drawLine(p1[j][i - 1] * BOX_SIZE_HALF, Vec3(1, 1, 1), p1[j][i] * BOX_SIZE_HALF, Vec3(1, 1, 1));
			}
			DUC->endLine();
		}
	}
}

Mat4 SPHSystemSimulator::getObj2World(const Vec3& position, const Vec3& size) {
	const auto &p = position, s = size;
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

	static const auto orientation = Quat(0, 0, 0, 1);
	return scale_mat * orientation.getRotMat() * translate_mat;
}

void SPHSystemSimulator::externalForcesCalculations(float timeElapsed) {
}

void SPHSystemSimulator::simulateTimestep(float timeStep) {
	if (m_Rain) {
		const auto toBeOrNotToBe = rand() % m_Rain_coefficient;
		if (toBeOrNotToBe == 1) {
			static const float eps = BOX_SIZE / 10;
			std::uniform_real_distribution<float> randPos(-BOX_SIZE_HALF + eps, BOX_SIZE_HALF - eps);
			const Real x = randPos(eng);
			const Real y = BOX_SIZE_HALF - eps;
			const Real z = randPos(eng);
			const auto pos = Vec3(x, y, z);
			const auto idx = posToGridCell(pos);
			if (obstacle_cell[idx] == 0)
				m_Particles.emplace_back(pos);
		}
	}
	computeDensityPressure();
	computeForces();
	integrate(timeStep);
	resolveCollision(timeStep);
}

void SPHSystemSimulator::notifyCaseChanged(int testCase) {
	m_Test_case = testCase;
	m_Particles.clear();
	switch (testCase) {
	case 0: {
		// Simplicity of Water
		m_Rain = false;
		city_colors_enabled = false;
		static const float eps = BOX_SIZE / 10;
		for (Real x = -BOX_SIZE_HALF + eps; x <= BOX_SIZE_HALF - eps; x += particleDiameter) {
			for (Real y = -BOX_SIZE_HALF + eps; y <= BOX_SIZE_HALF - eps; y += particleDiameter) {
				for (Real z = -BOX_SIZE_HALF + eps; z <= BOX_SIZE_HALF - eps; z += particleDiameter)
					m_Particles.emplace_back(Vec3(x, y, z));
			}
		}
		break;
	}
	case 1: {
		// Tsunami
		m_Rain = false;
		city_colors_enabled = true;
		for (Real x = -BOX_SIZE_HALF; x <= BOX_SIZE / 10; x += particleDiameter) {
			for (Real y = -BOX_SIZE_HALF; y <= BOX_SIZE / 4; y += particleDiameter) {
				for (Real z = -BOX_SIZE_HALF; z <= BOX_SIZE / 4; z += particleDiameter)
					m_Particles.emplace_back(Vec3(x, y, z));
			}
		}
		break;
	}
	case 2: {
		// Rainy day at the sea
		m_Rain = true;
		m_Rain_coefficient = 7;
		city_colors_enabled = false;
		static const float eps = BOX_SIZE / 10;
		for (Real x = -BOX_SIZE_HALF + eps; x <= BOX_SIZE_HALF - eps; x += particleDiameter) {
			for (Real y = -BOX_SIZE_HALF + eps; y <= BOX_SIZE / 4 - eps; y += particleDiameter) {
				for (Real z = -BOX_SIZE_HALF + eps; z <= BOX_SIZE_HALF - eps; z += particleDiameter)
					m_Particles.emplace_back(Vec3(x, y, z));
			}
		}
		break;
	}
	case 3: {
		// Rain and the city
		m_Rain = true;
		m_Rain_coefficient = 20;
		city_colors_enabled = true;
		break;
	}
	case 4: {
		// Overflowing pool
		m_Rain = false;
		city_colors_enabled = false;
		for (Real x = -BOX_SIZE_HALF; x <= -BOX_SIZE / 10; x += particleDiameter) {
			for (Real y = -BOX_SIZE_HALF; y <= BOX_SIZE / 2 - cell_size; y += particleDiameter) {
				for (Real z = -BOX_SIZE_HALF + BOX_SIZE / 10; z <= BOX_SIZE / 4; z += particleDiameter)
					m_Particles.emplace_back(Vec3(x, y, z));
			}
		}
		break;
	}
	default:
		break;
	}

	// Initialization for the occupancy grid
	obstacle_cell.clear();
	for (Real x = -BOX_SIZE_HALF; x <= BOX_SIZE_HALF; x += cell_size) {
		for (Real y = -BOX_SIZE_HALF; y <= BOX_SIZE_HALF; y += cell_size) {
			for (Real z = -BOX_SIZE_HALF; z <= BOX_SIZE_HALF; z += cell_size)
				obstacle_cell.emplace_back(0);
		}
	}
}

void SPHSystemSimulator::onClick(int x, int y) {
}

void SPHSystemSimulator::onMouse(int x, int y) {
}

void SPHSystemSimulator::computeDensityPressure() {
	max_pressure = -std::numeric_limits<Real>::max();
	min_pressure = std::numeric_limits<Real>::max();

	for (auto& pi : m_Particles) {
		pi.density = 0;
		for (const auto& pj : m_Particles) { pi.density += MASS * defaultKernel(pi.position - pj.position); }

		pi.old_pressures.push(pi.pressure);
		pi.sum_pressure += pi.pressure;
		if (pi.old_pressures.size() > 5) {
			pi.sum_pressure -= pi.old_pressures.front();
			pi.old_pressures.pop();
		}

		pi.pressure = GAS_CONST * (pi.density - REST_DENS);

		min_pressure = std::min(pi.pressure, min_pressure);
		max_pressure = std::max(pi.pressure, max_pressure);
	}
}

void SPHSystemSimulator::computeForces() {
	for (auto& pi : m_Particles) {
		Vec3 f_press(0, 0, 0), f_visc(0, 0, 0), f_surf(0, 0, 0), f_surf_normal(0, 0, 0);
		const auto pi_density_sq = pi.density * pi.density;

		for (auto& pj : m_Particles) {
			const auto distance = pi.position - pj.position;

			if (&pi != &pj) {
				f_press += pressureKernel_gradient(distance) * (pi.pressure / pi_density_sq + pj.pressure /
					(pj.density * pj.density)) * MASS;
				f_visc += (pj.velocity - pi.velocity) * (MASS / pj.density) * viscosityKernel_laplacian(distance);
			}

			f_surf_normal += defaultKernel_gradient(distance) * (MASS / pj.density);
			f_surf += (MASS / pj.density) * defaultKernel_laplacian(distance);
		}

		auto len = normalize(f_surf_normal);
		if (len >= THRESHOLD) f_surf = -(f_surf_normal * SURFACE_TENSION * f_surf);
		else f_surf = Vec3(0, 0, 0);

		f_press = -(f_press * pi.density);
		f_visc *= VISC;
		Vec3 f_grav = G * pi.density;

		if (surface_tension) { pi.force = f_press + f_visc + f_grav + f_surf; }
		else { pi.force = f_press + f_visc + f_grav; }
	}
}

void SPHSystemSimulator::integrate(float time_step) {
	for (auto& p : m_Particles) {
		p.velocity += time_step * p.force / p.density;
		p.position += time_step * p.velocity;
	}
}

void SPHSystemSimulator::fillOccupancyGrid(Vec3 pos, Vec3 size) {
	const auto eps = cell_size;
	for (Real x = pos.x - size.x - eps; x <= pos.x + size.x + eps; x += cell_size) {
		for (Real y = pos.y - size.y - eps; y <= pos.y + size.y + eps; y += cell_size) {
			for (Real z = pos.z - size.z - eps; z <= pos.z + size.z + eps; z += cell_size) {
				const int index = posToGridCell(Vec3(x, y, z));
				if (index >= 0) { obstacle_cell[index] = 1; }
			}
		}
	}
	for (Real x = pos.x - size.x; x <= pos.x + size.x; x += cell_size) {
		for (Real y = pos.y - size.y; y <= pos.y + size.y; y += cell_size) {
			for (Real z = pos.z - size.z; z <= pos.z + size.z; z += cell_size) {
				auto new_pos = Vec3(x, y, z);
				const int index = posToGridCell(new_pos);
				if (index >= 0) {
					occupied.emplace_back(new_pos + Vec3(cell_size, cell_size * 2, cell_size) / 2);
					city_colors.emplace_back(rand() % 4 == 1);
				}
			}
		}
	}
}

int SPHSystemSimulator::posToGridCell(Vec3 pos) const {
	auto cx = int(floor((pos.x + BOX_SIZE / 2) / cell_size));
	auto cy = int(floor((pos.y + BOX_SIZE / 2) / cell_size));
	auto cz = int(floor((pos.z + BOX_SIZE / 2) / cell_size));
	int index = cz * grid_size * grid_size + cy * grid_size + cx;
	return index;
}

void SPHSystemSimulator::resolveCollision(float time_step) {
	Vec3 contact_point;
	Vec3 unit_surface_normal;

	for (auto& p : m_Particles) {
		const int cell_index = posToGridCell(p.position);
		if (cell_index >= 0 && obstacle_cell[cell_index] == 1) {
			p.velocity = -p.velocity * RESTITUTION;
			p.position = p.position + p.velocity * time_step * 2;
		}

		if (detectCollision(p, contact_point, unit_surface_normal)) {
			const auto penetrationDepth = normalize(p.position - contact_point);
			const auto norm_vel = normalize(p.velocity);
			auto velocity = p.velocity * norm_vel;

			p.velocity = velocity - unit_surface_normal * (1 + RESTITUTION * penetrationDepth / (time_step * norm_vel))
				* dot(velocity, unit_surface_normal);
			p.position = contact_point;
		}
	}
}

std::pair<Vec3, Vec3> SPHSystemSimulator::wallContact(const Particle& p,
                                                      const std::tuple<uint8_t, uint8_t, uint8_t>& axes) {
	auto contact_point = p.position;
	const auto& [a, b, c] = axes;
	if (p.position[b] < -BOX_SIZE_HALF) contact_point[b] = -BOX_SIZE_HALF;
	else if (p.position[b] > BOX_SIZE_HALF) contact_point[b] = BOX_SIZE_HALF;
	if (p.position[c] < -BOX_SIZE_HALF) contact_point[c] = -BOX_SIZE_HALF;
	else if (p.position[c] > BOX_SIZE_HALF) contact_point[c] = BOX_SIZE_HALF;

	Vec3 normal_vec(0, 0, 0);

	if (p.position[a] < -BOX_SIZE_HALF) {
		contact_point[a] = -BOX_SIZE_HALF;
		normal_vec[a] = 1;
	}
	else {
		contact_point[a] = BOX_SIZE_HALF;
		normal_vec[a] = -1;
	}

	return std::make_pair(contact_point, normal_vec);
};


bool SPHSystemSimulator::detectCollision(const Particle& particle, Vec3& contactPoint, Vec3& unitSurfaceNormal) const {
	if (abs(particle.position.x) <= BOX_SIZE_HALF &&
		abs(particle.position.y) <= BOX_SIZE_HALF &&
		abs(particle.position.z) <= BOX_SIZE_HALF)
		return false;

	// 'unitSurfaceNormal' is based on the current position component with the largest absolute value
	const auto [x, y, z] = std::tuple{
		abs(particle.position.x), abs(particle.position.y), abs(particle.position.z)
	};
	if (z > std::max(x, y)) { std::tie(contactPoint, unitSurfaceNormal) = wallContact(particle, {2, 0, 1}); }
	else if (y > std::max(x, z)) { std::tie(contactPoint, unitSurfaceNormal) = wallContact(particle, {1, 0, 2}); }
	else { std::tie(contactPoint, unitSurfaceNormal) = wallContact(particle, {0, 1, 2}); }

	return true;
}


float SPHSystemSimulator::defaultKernel(Vec3 dist) {
	const auto dst = normalize(dist);
	if ((dst > SUPPORT_RADIUS)) return 0.0;
	return 315 / (64 * M_PI * powf(SUPPORT_RADIUS, 9.0f)) * powf(
		SUPPORT_RADIUS * SUPPORT_RADIUS - dst * dst, 3.0f);
}

Vec3 SPHSystemSimulator::defaultKernel_gradient(Vec3 dist) {
	const auto dst = normalize(dist);
	if ((dst > SUPPORT_RADIUS)) return Vec3(0, 0, 0);
	return -(dist * dst * (945 / (32 * M_PI * powf(SUPPORT_RADIUS, 9.0f))) * powf(
		SUPPORT_RADIUS * SUPPORT_RADIUS - dst * dst, 2.0f));
}

float SPHSystemSimulator::defaultKernel_laplacian(Vec3 dist) {
	const auto dst = normalize(dist);
	if ((dst > SUPPORT_RADIUS)) return 0.0;
	return -(945 / (32 * M_PI * powf(SUPPORT_RADIUS, 9.0f))) * (SUPPORT_RADIUS * SUPPORT_RADIUS - dst * dst) * (3
		* SUPPORT_RADIUS * SUPPORT_RADIUS - 7 * dst * dst);
}

Vec3 SPHSystemSimulator::pressureKernel_gradient(Vec3 dist) {
	const auto dst = normalize(dist);
	auto unit_vec = Vec3(1, 1, 1);
	normalize(unit_vec);
	if (dst > SUPPORT_RADIUS)
		return Vec3(0., 0., 0.);
	if (dst < 10e-5) // If ||r|| -> 0+
		return -(unit_vec * (45 / (M_PI * powf(SUPPORT_RADIUS, 6.0f))) * powf(SUPPORT_RADIUS - dst, 2.0f));
	return -(dist * (45 / (M_PI * powf(SUPPORT_RADIUS, 6.0f))) * powf(SUPPORT_RADIUS - dst, 2.0f));
}

float SPHSystemSimulator::viscosityKernel_laplacian(Vec3 dist) {
	const auto dst = normalize(dist);
	return (dst > SUPPORT_RADIUS) ? 0.0 : (45 / (M_PI * powf(SUPPORT_RADIUS, 6.0f))) * (SUPPORT_RADIUS - dst);
}
