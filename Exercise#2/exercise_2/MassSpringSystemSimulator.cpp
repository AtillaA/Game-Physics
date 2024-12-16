#include "MassSpringSystemSimulator.h"
#include <cmath>

using std::cout;
using std::endl;

MassSpringSystemSimulator::MassSpringSystemSimulator(): m_fMass(0), m_fStiffness(0), m_fDamping(0), m_iIntegrator(0),
                                                        m_mouse(),
                                                        m_trackmouse(),
                                                        m_oldtrackmouse() {

}

const char* MassSpringSystemSimulator::getTestCasesStr() { return "EULER,MIDPOINT"; }

void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass* DUC) {
	this->DUC = DUC;
	setMass(10.0f);
	setDampingFactor(1.0f);
	setStiffness(40.0f);
	applyExternalForce(Vec3(0, 0, 0));
	setIntegrator(EULER);
	gravity = true;

	int p0 = addMassPoint(Vec3(0.0, 0.0f, 0), Vec3(-1.0, 0.0f, 0), false);
	int p1 = addMassPoint(Vec3(0.0, 2.0f, 0), Vec3(1.0, 0.0f, 0), false);
	int p2 = addMassPoint(Vec3(1.0, 2.0f, 0), Vec3(1.0, -1.0f, 0), false);
	int p3 = addMassPoint(Vec3(0.0, 1.0f, 0), Vec3(1.0, -2.0f, 0), false);
	int p4 = addMassPoint(Vec3(1.0, 2.0f, 0), Vec3(1.0, 1.0f, 0), false);
	int p5 = addMassPoint(Vec3(1.5, 2.0f, 0), Vec3(-1.0, 1.0f, 0), false);
	int p6 = addMassPoint(Vec3(2.0, 2.0f, 0), Vec3(1.0, -1.0f, 0), false);
	int p7 = addMassPoint(Vec3(2.0, 1.0f, 0), Vec3(-1.0, 1.0f, 0), false);
	int p8 = addMassPoint(Vec3(1.5, 1.0f, 0), Vec3(1.5, 1.5f, 0), false);
	int p9 = addMassPoint(Vec3(1.5, 1.5f, 0), Vec3(1.5, 1.5f, 0), false);

	addSpring(p0, p1, 1.0);
	addSpring(p1, p2, 1.0);
	addSpring(p2, p3, 1.0);
	addSpring(p3, p4, 1.0);
	addSpring(p4, p5, 1.0);
	addSpring(p5, p6, 1.0);
	addSpring(p6, p7, 1.0);
	addSpring(p7, p8, 1.0);
	addSpring(p8, p9, 1.0);
	addSpring(p9, p0, 1.0);
};

void MassSpringSystemSimulator::reset() {
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
};

void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext) {
	std::mt19937 eng;
	std::uniform_real_distribution<float> randCol(0.0f, 1.0f);
	//std::uniform_real_distribution<float> randPos(-0.5f, 0.5f);

	for (int i = 0; i < points.size(); i++) {
		DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, 0.6 * Vec3(randCol(eng), randCol(eng), randCol(eng)));
		DUC->drawSphere(points[i].pose, Vec3(mp_size, mp_size, mp_size));
	}

	for (int j = 0; j < springs.size(); j++) {
		int mp1 = springs[j].mpoint1;
		int mp2 = springs[j].mpoint2;
		DUC->beginLine();
		DUC->drawLine(points[mp1].pose, Vec3(1, 1, 1), points[mp2].pose, Vec3(1, 1, 1));
		DUC->endLine();
	}
};

void MassSpringSystemSimulator::notifyCaseChanged(uint32_t testCase) {
	setIntegrator(testCase);
	cout << m_iIntegrator << endl;

	points.clear();
	springs.clear();
	forces.clear();
};

void MassSpringSystemSimulator::externalForcesCalculations(double timeElapsed) {

};

void MassSpringSystemSimulator::simulateTimestep(const double timeStep) {

	vector<Vec3> accelerations, tmp_accelerations;
	vector<PoseVel3D> tmp_points;

	switch (m_iIntegrator) {
		// handling different cases
	case EULER:
		accelerations = compute_accelerations(points);
		for (auto i = 0; i < points.size(); i++) {
			if (!points[i].is_fixed && points[i].pose.y >= -1 + mp_size) {
				points[i].pose += timeStep * points[i].vel;
				points[i].vel += timeStep * accelerations[i];
			}
		}
		break;
	case MIDPOINT:
		accelerations = compute_accelerations(points);
		for (auto i = 0; i < points.size(); i++) {
			const auto tmp_pose = points[i].pose + (0.5 * timeStep * points[i].vel);
			const auto tmp_vel = points[i].vel + (0.5 * timeStep * accelerations[i]);
			tmp_points.push_back(PoseVel3D{tmp_pose, tmp_vel, points[i].is_fixed});
		}

		tmp_accelerations = compute_accelerations(tmp_points);

		for (auto i = 0; i < points.size(); i++) {
			if (!points[i].is_fixed && points[i].pose.y >= -1 + mp_size) {
				points[i].pose += timeStep * tmp_points[i].vel;
				points[i].vel += timeStep * tmp_accelerations[i];
			}
		}
		break;

	default:
		break;
	}

};

void MassSpringSystemSimulator::onClick(int x, int y) {
	m_trackmouse.x = x;
	m_trackmouse.y = y;
};

void MassSpringSystemSimulator::onMouse(int x, int y) {
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
};

vector<Vec3> MassSpringSystemSimulator::compute_accelerations(vector<PoseVel3D> input_points) {

	vector<Vec3> accelerations;

	forces.clear();
	for (auto i = 0; i < input_points.size(); i++) { forces.push_back(Vec3(0.0, 0.0, 0.0)); }

	for (auto i = 0; i < springs.size(); i++) {
		const auto mp1 = springs[i].mpoint1;
		const auto mp2 = springs[i].mpoint2;
		const float current_length = sqrt(
			pow(input_points[mp1].pose.x - input_points[mp2].pose.x, 2) +
			pow(input_points[mp1].pose.y - input_points[mp2].pose.y, 2) +
			pow(input_points[mp1].pose.z - input_points[mp2].pose.z, 2));
		const auto d_normalize = (input_points[mp1].pose - input_points[mp2].pose) / current_length;
		const auto force = -m_fStiffness * (current_length - springs[i].initial_length) * d_normalize;

		forces[mp1] += force;
		forces[mp2] -= force;
	}

	for (int i = 0; i < input_points.size(); i++) {
		auto total_force = forces[i] - m_fDamping * input_points[i].vel;
		if (gravity) { total_force += Vec3(0, -9.8, 0); }
		accelerations.push_back(
			total_force / m_fMass
		);
	}

	return accelerations;
}

void MassSpringSystemSimulator::setMass(const float mass) { m_fMass = mass; }

void MassSpringSystemSimulator::setStiffness(const float stiffness) { m_fStiffness = stiffness; }

void MassSpringSystemSimulator::setDampingFactor(const float damping) { m_fDamping = damping; }

int MassSpringSystemSimulator::addMassPoint(const Vec3 position, const Vec3 velocity, const bool is_fixed) {
	const auto point = PoseVel3D{position, velocity, is_fixed};
	const int idx = points.size();
	points.push_back(point);
	forces.push_back(Vec3(0.0, 0.0, 0.0));
	return idx;
}

void MassSpringSystemSimulator::addSpring(const int mass_point1, const int mass_point2, const float initial_length) {
	const auto spring = Spring{mass_point1, mass_point2, initial_length};
	springs.push_back(spring);
}

int MassSpringSystemSimulator::getNumberOfMassPoints() const { return points.size(); }

int MassSpringSystemSimulator::getNumberOfSprings() const { return springs.size(); }

Vec3 MassSpringSystemSimulator::getPositionOfMassPoint(int index) { return points[index].pose; }

Vec3 MassSpringSystemSimulator::getVelocityOfMassPoint(int index) { return points[index].vel; }

void MassSpringSystemSimulator::applyExternalForce(Vec3 force) {

}
