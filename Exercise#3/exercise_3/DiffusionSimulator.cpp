#include "DiffusionSimulator.h"
#include "pcgsolver.h"
using namespace std;

DiffusionSimulator::DiffusionSimulator(): m_Mouse(), m_Track_mouse(), m_Old_track_mouse(), m_Grid_size({16, 16, 1}),
                                          m_Alpha(2.0) {
	m_iTestCase = 0;
	m_Vf_movable_object_pos = Vec3();
	m_Vf_movable_object_final_pos = Vec3();
	m_Vf_rotate = Vec3();
	// to be implemented
}

const char* DiffusionSimulator::getTestCasesStr() { return "Explicit_solver, Implicit_solver"; }

void DiffusionSimulator::reset() {
	m_Mouse.x = m_Mouse.y = 0;
	m_Track_mouse.x = m_Track_mouse.y = 0;
	m_Old_track_mouse.x = m_Old_track_mouse.y = 0;

}

void DiffusionSimulator::initUI(DrawingUtilitiesClass* duc) {
	this->DUC = duc;

	TwAddVarRW(DUC->g_pTweakBar, "nx", TW_TYPE_UINT32, &m_Grid_size.nx, "min=3");
	TwAddVarRW(DUC->g_pTweakBar, "ny", TW_TYPE_UINT32, &m_Grid_size.ny, "min=3");
	// TwAddVarRW(DUC->g_pTweakBar, "nz", TW_TYPE_UINT32, &m_Grid_size.nz, "min=1");
	TwAddVarRW(DUC->g_pTweakBar, "alpha", TW_TYPE_FLOAT, &m_Alpha, "min=0.01 step=0.01");
}

void DiffusionSimulator::notifyCaseChanged(const int test_case) {
	m_iTestCase = test_case;
	m_Vf_movable_object_pos = Vec3(0, 0, 0);
	m_Vf_rotate = Vec3(0, 0, 0);
	//
	//to be implemented
	//
	switch (m_iTestCase) {
	case 0:
		cout << "Explicit solver!\n";
		m_T = std::shared_ptr<Grid>();
		break;
	case 1:
		cout << "Implicit solver!\n";
		m_T = std::shared_ptr<Grid>();
		break;
	default:
		cout << "Empty Test!\n";
		break;
	}
}

std::shared_ptr<Grid> DiffusionSimulator::diffuseTemperatureExplicit(double time_step) {
	//add your own parameters
	if (m_T == nullptr) { m_T = std::make_shared<Grid>(m_Grid_size.nx, m_Grid_size.ny, m_Grid_size.nz); }

	// to be implemented
	const auto dx = 1.0; // FIXME
	const auto dy = dx;
	const auto dt = time_step;
	const auto alpha = m_Alpha; // FIXME
	const auto F = alpha * dt / (4 * dx * dy);

	auto new_grid = std::make_shared<Grid>(m_T->m_Width, m_T->m_Height, m_T->m_Depth);
	auto& u = new_grid->m_Values;
	const auto& u_1 = m_T->m_Values;
	const auto& Nx = m_T->m_Width;
	const auto& Ny = m_T->m_Height;

	assert(Nx >= 1);
	for (unsigned i = 1; i < Nx - 1; ++i) {
		for (unsigned j = 1; j < Ny - 1; ++j) {
			//u[i][j][0] = u_1[i][j][0] + F * (u_1[i - 1][j][0] - 2 * u_1[i][j][0] + u_1[i + 1][j][0]);
			u[i][j][0] = u_1[i][j][0] +
				F * (u_1[i + 1][j][0] + u_1[i - 1][j][0] + u_1[i][j + 1][0] + u_1[i][j - 1][0] - 4 * u_1[i][j][0]);

			//	u[i][j][0] = u_1[i][j][0] +
			//		F * (u_1[i + 1][j + 1][0] - u_1[i - 1][j + 1][0] - u_1[i + 1][j - 1][0] + u_1[i - 1][j - 1][0]);
		}
	}

	//make sure that the temperature in boundary cells stays zero
	for (unsigned i = 0; i < Nx; ++i) { u[i][0][0] = u[i][Ny - 1][0] = 0.0; }
	for (unsigned j = 1; j < Ny - 1; ++j) { u[0][j][0] = u[Nx - 1][j][0] = 0.0; }

	return new_grid;
}

void setupB(std::vector<Real>& b, uint32_t width, uint32_t height,
            const std::vector<std::vector<std::vector<double>>>& u_1) {
	//add your own parameters
	// to be implemented
	//set vector B[sizeX*sizeY]
	for (uint32_t i = 0; i < height; i++) {
		for (uint32_t j = 0; j < width; j++) {
			if (i == 0 || j == 0 || i == height - 1 || j == width - 1) { b.at(i * width + j) = 0; }
			else { b.at(i * width + j) = u_1[j][i][0]; }
		}
	}
}

void fillT(std::shared_ptr<Grid> T, std::vector<Real>& x) {
	//add your own parameters
	// to be implemented
	//fill T with solved vector x
	//make sure that the temperature in boundary cells stays zero
	for (uint32_t i = 0; i < T->m_Width; ++i) {
		for (uint32_t j = 0; j < T->m_Height; ++j) {
			// ...
			T->m_Values[i][j][0] = // ... 
				x[j * T->m_Width + i];
		}
	}

	for (uint32_t i = 0; i < T->m_Width; ++i) {
		T->m_Values[i][0][0] = 0;
		T->m_Values[i][T->m_Height - 1][0] = 0;
	}

	for (uint32_t j = 0; j < T->m_Height; ++j) {
		T->m_Values[0][j][0] = 0;
		T->m_Values[T->m_Width - 1][j][0] = 0;
	}
}

void setupA(SparseMatrix<Real>& A, uint32_t width, uint32_t height, double factor) {
	//add your own parameters
	// to be implemented
	//setup Matrix A[sizeX*sizeY*sizeZ, sizeX*sizeY*sizeZ]
	// set with:  A.set_element( index1, index2 , value );
	// if needed, read with: A(index1, index2);
	// avoid zero rows in A -> set the diagonal value for boundary cells to 1.0
	for (uint32_t i = 0; i < A.n; ++i) { A.set_element(i, i, 1); }

	for (auto i = 1; i < height - 1; i++) {
		for (auto j = 1; j < width - 1; j++) {
			const auto u_11 = width * i + j;
			const auto u_01 = width * (i - 1) + j;
			const auto u_21 = width * (i + 1) + j;
			const auto u_10 = width * i + (j - 1);
			const auto u_12 = width * i + (j + 1);
			A.set_element(u_11, u_21, -factor);
			A.set_element(u_11, u_12, -factor);
			A.set_element(u_11, u_01, -factor);
			A.set_element(u_11, u_10, -factor);
			A.set_element(u_11, u_11, 1 + 4 * factor);
		}
	}
}


void DiffusionSimulator::diffuseTemperatureImplicit(double time_step) {
	//add your own parameters
	// solve A T = b
	// to be implemented
	if (m_T == nullptr) { m_T = std::make_shared<Grid>(m_Grid_size.nx, m_Grid_size.ny, m_Grid_size.nz); }
	const auto N = m_T->m_Width * m_T->m_Height; //N = sizeX*sizeY*sizeZ
	auto a = SparseMatrix<Real>(N);
	auto b = std::vector<Real>(N);

	const auto dx = 1.0; // FIXME
	const auto dt = time_step;
	const auto alpha = m_Alpha; // FIXME
	const auto F = alpha * dt / (dx * dx);

	setupA(a, m_T->m_Width, m_T->m_Height, F);

	const auto& u_1 = m_T->m_Values;

	setupB(b, m_T->m_Width, m_T->m_Height, u_1);

	// perform solve
	const auto pcg_target_residual = 1e-05;
	const auto pcg_max_iterations = 1000;
	auto ret_pcg_residual = 1e10;
	auto ret_pcg_iterations = -1;

	SparsePCGSolver<Real> solver;
	solver.set_solver_parameters(pcg_target_residual, pcg_max_iterations, 0.97, 0.25);

	std::vector<Real> x(N);
	for (auto j = 0; j < N; ++j) { x[j] = 0.; }

	// preconditioners: 0 off, 1 diagonal, 2 incomplete cholesky
	solver.solve(a, b, x, ret_pcg_residual, ret_pcg_iterations, 0);
	// x contains the new temperature values
	fillT(m_T, x); //copy x to T
}


void DiffusionSimulator::simulateTimestep(float time_step) {
	// to be implemented
	// update current setup for each frame
	switch (m_iTestCase) {
	case 0:
		m_T = diffuseTemperatureExplicit(time_step);
		break;
	case 1:
		diffuseTemperatureImplicit(time_step);
		break;
	default: break;
	}
}

void DiffusionSimulator::drawObjects() {
	// to be implemented
	//visualization
	if (m_T == nullptr) { return; }

	const auto sphere_size = 0.1;
	for (unsigned i = 0; i < m_T->m_Width; ++i) {
		for (unsigned j = 0; j < m_T->m_Height; ++j) {
			for (unsigned k = 0; k < m_T->m_Depth; ++k) {
				const auto& value = m_T->m_Values[i][j][k];

				Vec3 color;
				if (value > 0.0) { color = Vec3(value, value, value); }
				else { color = Vec3(-value, -value * 0.4, -value * 0.4); }

				DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100,
				                   0.8 * color);
				DUC->drawSphere(
					(Vec3(i, j, k) - (Vec3(m_T->m_Width, m_T->m_Height, m_T->m_Depth) * 0.5)) * sphere_size * 1.4,
					Vec3(sphere_size, sphere_size, sphere_size));
			}
		}
	}
}


void DiffusionSimulator::drawFrame(ID3D11DeviceContext* pd3d_immediate_context) { drawObjects(); }

void DiffusionSimulator::onClick(const int x, const int y) {
	m_Track_mouse.x = x;
	m_Track_mouse.y = y;
}

void DiffusionSimulator::onMouse(int x, int y) {
	m_Old_track_mouse.x = x;
	m_Old_track_mouse.y = y;
	m_Track_mouse.x = x;
	m_Track_mouse.y = y;
}
