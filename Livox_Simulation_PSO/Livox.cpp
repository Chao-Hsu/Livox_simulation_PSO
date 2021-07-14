#include "Livox.h"

unsigned THREADS;
char flag_cross_section;


unsigned nMid70 = 0;
unsigned nHorizon = 0;
unsigned nSensor = nMid70 + nHorizon;

Livox::Livox()
{
	ray = nullptr;

	ox = 0.0f;
	oy = 0.0f;
	oz = 0.0f;
	roll = 0.0f;
	pitch = 0.0f;

	nPts = 0U;
}
Livox::Livox(float tx, float ty, float tz, float rx, float ry)
{
	ox = tx;
	oy = ty;
	oz = tz;
	roll = rx;
	pitch = ry;

	nPts = 0U;

	ray = nullptr;
}
void Livox::print()const
{
	for (size_t i = 0; i < nPts; i++)
	{
		std::cout << ray[i] << "\n";
	}
}
unsigned Livox::get_nPts() { return nPts; }
float Livox::d_to_r(float degree) { return degree / 180.0f * EIGEN_PI; }
void Livox::visualiation(std::vector<Livox>& Sensors)
{
	pcl::visualization::PCLVisualizer viewer("cloud");
	viewer.setBackgroundColor(0, 0, 0);

	std::unique_ptr<pcl::PointCloud<pcl::PointXYZL>[]> clouds(new pcl::PointCloud<pcl::PointXYZL>[Sensors.size()]);
	unsigned countPts = 0;
	for (size_t i = 0; i < Sensors.size(); i++)
	{
		for (size_t j = 0; j < Sensors[i].get_nPts(); j++)
		{
			clouds[i].emplace_back(cloud.points[j + countPts]);
		}
		countPts += Sensors[i].get_nPts();
	}

	std::unique_ptr<pcl::PointCloud<pcl::PointXYZL>::Ptr[]> clouds_ptr(new pcl::PointCloud<pcl::PointXYZL>::Ptr[Sensors.size()]);
	for (size_t i = 0; i < Sensors.size(); i++)
	{
		clouds_ptr[i] = clouds[i].makeShared();
	}

	pcl::PassThrough<pcl::PointXYZL> passTrans;
	for (size_t i = 0; i < Sensors.size(); i++)
	{
		passTrans.setInputCloud(clouds_ptr[i]);
		passTrans.setFilterFieldName("z");
		passTrans.setFilterLimits(0.0f, 50.0f);
		passTrans.filter(*clouds_ptr[i]);
	}

	//pcl::VoxelGrid<pcl::PointXYZL> voxelGrid;
	//for (size_t i = 0; i < Sensors.size(); i++)
	//{
	//	voxelGrid.setInputCloud(clouds_ptr[i]);
	//	voxelGrid.setLeafSize(0.1f, 0.1f, 0.1f);
	//	voxelGrid.filter(*clouds_ptr[i]);
	//}

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZL> color1(clouds_ptr[0], 255, 0, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZL> color2(clouds_ptr[1], 255, 153, 18);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZL> color3(clouds_ptr[2], 0, 0, 255);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZL> color4(clouds_ptr[3], 0, 255, 0);
	viewer.addPointCloud<pcl::PointXYZL>(clouds_ptr[0], color1, "Mid70_1");
	viewer.addPointCloud<pcl::PointXYZL>(clouds_ptr[1], color2, "Mid70_2");
	viewer.addPointCloud<pcl::PointXYZL>(clouds_ptr[2], color3, "Horizon_1");
	viewer.addPointCloud<pcl::PointXYZL>(clouds_ptr[3], color4, "Horizon_2");
	//viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample");

	while (!viewer.wasStopped()) {
		viewer.spinOnce(10);
	}
}


Mid::Mid() :Livox() { FoV = 0.0f; }
unsigned Mid::nPtsPerSec = 100000;
float Mid::dt = 1.0f / Mid::nPtsPerSec;
float Mid::rpm2 = -4664.0f;
float Mid::w2 = Mid::rpm2 / 60.0f * 2.0f * EIGEN_PI;
Mid::Mid(float tx, float ty, float tz, float rx, float ry, float fov) : Livox(tx, ty, tz, rx, ry)
{
	nPts = scanning_time * nPtsPerSec;
	ray = new Eigen::Vector3d[nPts + 1U];

	FoV = fov;

	Eigen::Vector3d O(ox, oy, oz);
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

	Eigen::AngleAxisd X(d_to_r(-roll), Eigen::Vector3d::UnitX());
	Eigen::AngleAxisd Y(d_to_r(-pitch), Eigen::Vector3d::UnitY());
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
		Cylinder::cylinder(ray, nPts + 1U);
		break;

	case choose_arch:
		Arch::arch(ray, nPts + 1U);
		break;

	default:
		break;
	}
}

Mid70::Mid70() : Mid() {}
Mid70::Mid70(float tx, float ty, float tz, float rx, float ry) : Mid(tx, ty, tz, rx, ry, 70.4f)
{
}
Mid70::Mid70(std::vector<float>::iterator b) : Mid(*b, *(b + 1), *(b + 2), *(b + 3), *(b + 4), 70.4f)
{
}
Mid70::Mid70(float* ptr) : Mid(*ptr, *(ptr + 1), *(ptr + 2), *(ptr + 3), *(ptr + 4), 70.4f)
{
}

Mid40::Mid40() : Mid() {}
Mid40::Mid40(float tx, float ty, float tz, float rx, float ry) : Mid(tx, ty, tz, rx, ry, 38.4f)
{
}
Mid40::Mid40(std::vector<float>::iterator b) : Mid(*b, *(b + 1), *(b + 2), *(b + 3), *(b + 4), 38.4f)
{
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
unsigned Horizon::nPtsPerSec = 40000;
float Horizon::dt = 1.0f / Horizon::nPtsPerSec;
float Horizon::rpm2 = -rpm1;
float Horizon::w2 = Horizon::rpm2 / 60.0f * 2.0f * EIGEN_PI;
float Horizon::rpm3 = 600.0f;
float Horizon::w3 = rpm3 / 60.0f * 2.0f * EIGEN_PI;
float Horizon::n1 = 1.5f;
float Horizon::n2 = 1.5f;
float Horizon::alpha1 = d_to_r(20.0f);
float Horizon::alpha2 = d_to_r(20.0f);
float Horizon::delta1 = (Horizon::n1 - 1.0f) * Horizon::alpha1;
float Horizon::delta2 = (Horizon::n2 - 1.0f) * Horizon::alpha2;
float Horizon::rotation_ratio = 0.1f;
float Horizon::translation_ratio = Horizon::rotation_ratio * 0.095f;
float Horizon::FoV_h = 81.7f;
float Horizon::FoV_v = 25.1f;
Horizon::Horizon(float tx, float ty, float tz, float rx, float ry, float rz) : Livox(tx, ty, tz, rx, ry)
{
	Horizon::nPts = scanning_time * Horizon::nPtsPerSec;
	ray = new Eigen::Vector3d[nPts * 6U + 1U];

	yaw = rz;

	Eigen::Vector3d O(ox, oy, oz);
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
	nPts *= 6;

	//rotation
	Eigen::AngleAxisd X(d_to_r(-roll), Eigen::Vector3d::UnitX());
	Eigen::AngleAxisd Y(d_to_r(-pitch), Eigen::Vector3d::UnitY());
	Eigen::AngleAxisd Z(d_to_r(-yaw), Eigen::Vector3d::UnitZ());
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


	//intersection
	switch (flag_cross_section)
	{
	case choose_cylinder:
		Cylinder::cylinder(ray, nPts + 1U);
		break;
	case choose_arch:
		Arch::arch(ray, nPts + 1U);
		break;
	default:
		break;
	}
}
Horizon::Horizon(std::vector<float>::iterator b) : Livox(*b, *(b + 1), *(b + 2), *(b + 3), *(b + 4))
{
	Horizon::nPts = scanning_time * Horizon::nPtsPerSec;
	ray = new Eigen::Vector3d[nPts * 6U + 1U];

	yaw = *(b + 5);

	Eigen::Vector3d O(ox, oy, oz);
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
	nPts *= 6;

	//rotation
	Eigen::AngleAxisd X(d_to_r(-roll), Eigen::Vector3d::UnitX());
	Eigen::AngleAxisd Y(d_to_r(-pitch), Eigen::Vector3d::UnitY());
	Eigen::AngleAxisd Z(d_to_r(-yaw), Eigen::Vector3d::UnitZ());
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


	//intersection
	switch (flag_cross_section)
	{
	case choose_cylinder:
		Cylinder::cylinder(ray, nPts + 1U);
		break;
	case choose_arch:
		Arch::arch(ray, nPts + 1U);
		break;
	default:
		break;
	}
}
Horizon::Horizon(float* ptr) : Livox(*ptr, *(ptr + 1), *(ptr + 2), *(ptr + 3), *(ptr + 4))
{
	Horizon::nPts = scanning_time * Horizon::nPtsPerSec;
	ray = new Eigen::Vector3d[nPts * 6U + 1U];

	yaw = *(ptr + 5);

	Eigen::Vector3d O(ox, oy, oz);
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
	nPts *= 6;

	//rotation
	Eigen::AngleAxisd X(d_to_r(-roll), Eigen::Vector3d::UnitX());
	Eigen::AngleAxisd Y(d_to_r(-pitch), Eigen::Vector3d::UnitY());
	Eigen::AngleAxisd Z(d_to_r(-yaw), Eigen::Vector3d::UnitZ());
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


	//intersection
	switch (flag_cross_section)
	{
	case choose_cylinder:
		Cylinder::cylinder(ray, nPts + 1U);
		break;
	case choose_arch:
		Arch::arch(ray, nPts + 1U);
		break;
	default:
		break;
	}
}
