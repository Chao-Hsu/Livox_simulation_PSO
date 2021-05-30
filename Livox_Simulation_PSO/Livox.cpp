#include "Livox.h"
#include "cross_section.h"

constexpr auto SCANNING_TIME = 10U;
int nMid70 = 0;
int nHorizon = 0;
int THREADS = 12;
char flag_cross_section = choose_cylinder;

Livox::Livox()
{
	ray = nullptr;

	scalar = 0.0f;
	ox = 0.0f;
	oy = 0.0f;
	oz = 0.0f;
	roll = 0.0f;
	pitch = 0.0f;

	scanning_time = 0;
	nPtsPerSec = 0;
	nPts = 0;

	dt = 0.0f;
	rpm1 = 0.0f;
	rpm2 = 0.0f;
	w1 = 0.0f;
	w2 = 0.0f;

	//particle = 0;
	//iteration = 0.0f;
	//limit = 0.0f;
	//W = 0.0f;
	//C1 = 0.0f;
	//C2 = 0.0f;

	//para_length = 0;
	//position = nullptr;
	//velocity = nullptr;
	//p_best_position = nullptr;
	//g_best_position = nullptr;
	//fitness = nullptr;
	//p_best_fitness = nullptr;
	//g_best_fitness = 0.0f;
}

Livox::Livox(float tx, float ty, float tz, float rx, float ry)
{
	scalar = 100.0f;
	ox = tx;
	oy = ty;
	oz = tz;
	roll = rx;
	pitch = ry;

	scanning_time = SCANNING_TIME;
	nPtsPerSec = 100000;
	nPts = scanning_time * nPtsPerSec;

	dt = 1.0f / nPtsPerSec;
	rpm1 = 7294.0f;
	rpm2 = 4664.0f;
	w1 = rpm1 / 60.0f * 2.0f * EIGEN_PI;
	w2 = rpm2 / 60.0f * 2.0f * EIGEN_PI;

	ray = new Eigen::Vector3d[nPts + 1U];
}

Livox::Livox(float tx, float ty, float tz, float rx, float ry, float rz)
{
	scalar = 100.0f;
	ox = tx;
	oy = ty;
	oz = tz;
	roll = rx;
	pitch = ry;

	scanning_time = SCANNING_TIME;
	nPtsPerSec = 40000;
	nPts = scanning_time * nPtsPerSec;

	dt = 1.0f / nPtsPerSec;
	rpm1 = 7294.0f;
	rpm2 = -rpm1;
	w1 = rpm1 / 60.0f * 2.0f * EIGEN_PI;
	w2 = rpm2 / 60.0f * 2.0f * EIGEN_PI;

	ray = new Eigen::Vector3d[nPts * 6U + 1U];
}

//float Livox::get_scalar() { return scalar; }
//float Livox::get_ox() { return ox; }
//float Livox::get_oy() { return oy; }
//float Livox::get_oz() { return oz; }
//float Livox::get_roll() { return roll; }
//float Livox::get_pitch() { return pitch; }
//unsigned Livox::get_nPts() { return nPts; }
//float Livox::get_dt() { return dt; }
//float Livox::get_w1() { return w1; }
//float Livox::get_w2() { return w2; }
//Eigen::Vector3d* Livox::get_ray() { return ray; }

void Livox::print()const
{
	for (size_t i = 0; i < nPts; i++)
	{
		std::cout << ray[i] << "\n";
	}
}

Mid::Mid() :Livox() { FoV = 0.0f; }

Mid::Mid(float tx, float ty, float tz, float rx, float ry) : Livox(tx, ty, tz, rx, ry) { FoV = 0.0f; }



Mid70::Mid70() : Mid() {}

Mid70::Mid70(float tx, float ty, float tz, float rx, float ry) : Mid(tx, ty, tz, rx, ry)
{
	++nMid70;
	FoV = 70.4;

	//Eigen::Vector3d* ray = get_ray();
	Eigen::Vector3d O(tx, ty, tz);
	ray[0] = O;

	float trans = scalar * tan(FoV / 2.0f / 180.0f * EIGEN_PI) / 2.0f;
#pragma omp parallel for num_threads(THREADS) schedule(dynamic, nPts/THREADS)
	for (unsigned n = 1; n <= nPts; n++)
	{
		float a1 = n * dt * w1, a2 = n * dt * w2;
		float X = (cos(a1) + cos(a2)) * trans, Y = (sin(a1) + sin(a2)) * trans;
		Eigen::Vector3d tmpVec(X, Y, scalar);
		ray[n] = tmpVec;
	}

	Eigen::AngleAxisd X(d_to_r(-rx), Eigen::Vector3d::UnitX());
	Eigen::AngleAxisd Y(d_to_r(-ry), Eigen::Vector3d::UnitY());
#pragma omp parallel for num_threads(THREADS) schedule(dynamic, nPts/THREADS)
	for (unsigned n = 1; n <= nPts; n++)
	{
		ray[n] = Y.toRotationMatrix() * X.toRotationMatrix() * ray[n];
	}

#pragma omp parallel for num_threads(THREADS) schedule(dynamic, nPts/THREADS)
	for (unsigned n = 1; n <= nPts; n++)
	{
		ray[n] += O;
	}


	//intersection
	switch (flag_cross_section)
	{
	case choose_cylinder:
		cylinder(ray, nPts + 1U);
		break;

	case choose_arch:
		arch(ray, nPts + 1U);
		break;

	default:
		break;
	}
}

Horizon::Horizon() : Livox()
{
	yaw = 0.0f;

	rpm3 = 0.0f;
	w3 = 0.0f;
	n1 = 0.0f;
	n2 = 0.0f;
	alpha1 = 0.0f;
	alpha2 = 0.0f;
	delta1 = 0.0f;
	delta2 = 0.0f;
	rotation_ratio = 0.0f;
	translation_ratio = 0.0f;
	FoV_h = 0.0f;
	FoV_v = 0.0f;
}

Horizon::Horizon(float tx, float ty, float tz, float rx, float ry, float rz) : Livox(tx, ty, tz, rx, ry, rz)
{
	++nHorizon;

	yaw = rz;
	rpm3 = 6000.0f;
	w3 = rpm3 / 60.0f * 2.0f * EIGEN_PI;
	n1 = 1.5f;
	n2 = 1.5f;
	alpha1 = d_to_r(20.0f);
	alpha2 = d_to_r(20.0f);
	delta1 = (n1 - 1.0f) * alpha1;
	delta2 = (n2 - 1.0f) * alpha2;
	rotation_ratio = 0.1f;
	translation_ratio = rotation_ratio * 0.095f;
	FoV_h = 81.7f;
	FoV_v = 25.1f;

	//Eigen::Vector3d* ray = get_ray;
	Eigen::Vector3d O(tx, ty, tz);
	ray[0] = O;
	float transX = scalar * 1.0f / 0.516093285f * tan(d_to_r(FoV_h / 2.0f));
	float transY = scalar * 1.0f / 0.134329364f * tan(d_to_r(FoV_v / 2.0f));

	//generate vectors
	float A21 = sin(alpha1) * sin(alpha1) + cos(alpha1) * sqrt(n1 * n1 - sin(alpha1) * sin(alpha1));
	float A2 = cos(alpha2) * std::sqrt(n2 * n2 - n1 * n1 + A21 * A21);
#pragma omp parallel for num_threads(THREADS) schedule(dynamic, nPts/THREADS)
	for (unsigned n = 1; n <= nPts; n++)
	{
		float a1 = n * dt * w1, a2 = n * dt * w2, a3 = n * dt * w3;
		float A3 = sin(alpha1) * sin(alpha2) * (cos(alpha1) - sqrt(n1 * n1 - sin(alpha1) * sin(alpha1))) * cos(a1 - a2);
		float A1 = sqrt(1.0f - n2 * n2 + (A2 + A3) * (A2 + A3));
		float A = A1 - A2 - A3;
		float K = sin(alpha1) * (cos(alpha1) - sqrt(n1 * n1 - sin(alpha1) * sin(alpha1))) * cos(a1) + A * sin(alpha2) * cos(a2);
		float L = sin(alpha1) * (cos(alpha1) - sqrt(n1 * n1 - sin(alpha1) * sin(alpha1))) * sin(a1) + A * sin(alpha2) * sin(a2);
		float M = -sqrt(n2 * n2 - n1 * n1 + A21 * A21) - A * cos(alpha2);
		float X = K / M + cos(a3) * rotation_ratio, Y = L / M + sin(a3) * rotation_ratio;

		X *= transX;
		for (unsigned j = 0; j < 6; j++)
		{
			auto tmp_Y = (Y + translation_ratio * j - 0.023739453f) * transY;
			Eigen::Vector3d tmp_v(X, tmp_Y, scalar);
			ray[n + nPts * j] = tmp_v;
		}
	}

	//rotation
	Eigen::AngleAxisd X(d_to_r(-rx), Eigen::Vector3d::UnitX());
	Eigen::AngleAxisd Y(d_to_r(-ry), Eigen::Vector3d::UnitY());
	Eigen::AngleAxisd Z(d_to_r(-rz), Eigen::Vector3d::UnitZ());
#pragma omp parallel for num_threads(THREADS) schedule(dynamic, nPts/THREADS)
	for (unsigned n = 1; n <= nPts; n++)
	{
		ray[n] = Z.toRotationMatrix() * Y.toRotationMatrix() * X.toRotationMatrix() * ray[n];
	}

	//translation
#pragma omp parallel for num_threads(THREADS) schedule(dynamic, nPts/THREADS)
	for (unsigned n = 1; n <= nPts; n++)
	{
		ray[n] += O;
	}

	nPts *= 6;

	//intersection
	switch (flag_cross_section)
	{
	case choose_cylinder:
		cylinder(ray, nPts + 1U);
		break;
	case choose_arch:
		arch(ray, nPts + 1U);
		break;
	default:
		break;
	}
}
