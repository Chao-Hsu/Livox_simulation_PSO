#include "cross_section.h"

pcl::PointCloud<pcl::PointXYZL> cloud;
pcl::PointCloud<pcl::PointXYZL> cloud_cube;

Cross_Section::Cross_Section() {}
std::vector<unsigned>::iterator Cross_Section::check_continue(std::vector<unsigned>::iterator i)
{
	if (*(i + 1) - *i != 1) return i;
	else return check_continue(i + 1);
}
unsigned Cross_Section::cube_x = 0U;
unsigned Cross_Section::cube_y = 0U;
unsigned Cross_Section::cube_z = 0U;
unsigned Cross_Section::nCube = 0U;
unsigned* Cross_Section::cube = nullptr;

Cylinder::Cylinder() {}

void Cylinder::cylinder(Eigen::Vector3d*& v, unsigned length) {
	cloud.width = length - 1 + cloud.size();
	cloud.height = 1;
	cloud.reserve(cloud.width * cloud.height);

	float ox = v[0].x(), oy = v[0].y(), oz = v[0].z();
	for (unsigned i = 1; i <= length; i++)
	{
		float tmpX = 0.0f, tmpY = 0.0f, tmpZ = 0.0f;
		float bx = v[i].x(), by = v[i].y(), bz = v[i].z();
		//Ax+By+C, x^2+y^2=r^2
		float A = (bx - ox) * (bx - ox) + (by - oy) * (by - oy);
		float B = 2.0f * (ox * (bx - ox) + oy * (by - oy));
		float C = ox * ox + oy * oy - r * r;
		float D = B * B - 4 * A * C;
		if (D > 0.0f)
		{
			float t1 = (-B + sqrt(D)) / (A * 2.0f)/*, t2 = (-B - sqrt(D)) / (A * 2.0f)*/;
			float tmpX1 = ox + (bx - ox) * t1/*, tmpX2 = ox + (bx - ox) * t2*/;
			float tmpY1 = oy + (by - oy) * t1/*, tmpY2 = oy + (by - oy) * t2*/;
			float tmpZ1 = oz + (bz - oz) * t1/*, tmpZ2 = oz + (bz - oz) * t2*/;
			if (tmpY1 >= h)
			{
				tmpX = tmpX1;
				tmpY = tmpY1;
				tmpZ = tmpZ1;
			}
			else if (tmpY1 - h < eps)
			{
				//float t = oy / (oy - by);
				float t = (h - oy) / (by - oy);
				tmpX = ox + (bx - ox) * t;
				tmpY = h;
				tmpZ = oz + (bz - oz) * t;
			}
			pcl::PointXYZL tmp{ tmpX,tmpY,tmpZ };
			cloud.emplace_back(tmp);
		}
	}
}

float Cylinder::statistics(bool print)
{
	cloud_cube.clear();
	cloud_cube.reserve((cube_x + cube_y) * 2 * cube_z);
	Cross_Section::cube_x = (unsigned)(r * 2.0f * scale) + 1U;
	Cross_Section::cube_y = (unsigned)((r + h) * scale) + 1U;
	Cross_Section::cube_z = (unsigned)(seg * scale) + 1U;
	Cross_Section::nCube = cube_x * cube_y * cube_z;
	Cross_Section::cube = new unsigned[nCube];

	unsigned cloud_size = cloud.size();
	if (print)
	{
		std::cout << "------------------------------------------------------------------------------\n"
			<< "Number of points = " << cloud_size << endl;
		std::cout << "Number of cubes = " << cube_x << " * " << cube_y << " * " << cube_z << " = " << nCube
			<< "\nSegmentation: " << std::fixed << setprecision(2) << end - start << "m, precision: " << precision << "m\n";
	}

#pragma omp parallel for num_threads(THREADS) schedule(dynamic, nCube/THREADS)
	for (unsigned i = 0; i < nCube; i++)
	{
		cube[i] = 0U;
	}

#pragma omp parallel for num_threads(THREADS) schedule(dynamic, cloud_size/THREADS)
	for (unsigned i = 0; i < cloud_size; i++)
	{
		if (cloud.points.at(i).z <= end && cloud.points.at(i).z >= start)
		{
			unsigned _x = (unsigned)((cloud.points.at(i).x + r) * scale);
			unsigned _y = (unsigned)((cloud.points.at(i).y + h) * scale);
			unsigned _z = (unsigned)((cloud.points.at(i).z - start) * scale);
			++cube[_z * cube_x * cube_y + _x * cube_y + _y];
		}
	}


	unsigned long long sCube = 0, sPts = 0, sN = 0, sVar = 0;

	float threshold = precision * sqrt(2.0) * 0.9f;
	unsigned startScale = start * scale, endScale = end * scale;
	unsigned nCompleted = 0, countCompleted = 0;
	std::vector<unsigned> segCompleted;
	segCompleted.reserve(cube_z);

	for (unsigned k = 0; k < cube_z; k++)
	{
		float z_ = (float)(k + start * scale) * precision;
		unsigned completed = 0;
		unsigned cubeCompleted = 0;
		for (unsigned i = 0; i < cube_x; i++)
		{
			float x_ = (float)(i - r * scale) * precision;
			for (unsigned j = 0; j < cube_y; j++)
			{
				float y_ = (float)(j - h * scale) * precision;
				float r_after = (x_ < 0.0f)
					? sqrt((x_ + precision) * (x_ + precision) + y_ * y_)
					: sqrt(x_ * x_ + y_ * y_);
				if ((int&)y_ <= (int&)h || (((int&)r >= (int&)r_after) && abs(r - r_after) - threshold <= eps))
				{
					if (print) {
						pcl::PointXYZL tmp{ x_,y_,z_ };
						cloud_cube.emplace_back(tmp);
					}
					++sCube;
					++cubeCompleted;
					auto n = (unsigned long long)cube[k * cube_x * cube_y + i * cube_y + j];
					if (n)
					{
						sVar += n * n;
						sPts += n;
						++sN;
						++completed;
					}
				}
			}
		}
		if ((float)completed / (float)cubeCompleted >= 0.9f)
		{
			++countCompleted;
			segCompleted.emplace_back((unsigned)(z_ * scale));
		}
		else
		{
			countCompleted = 0U;
		}
		if (countCompleted > nCompleted)nCompleted = countCompleted;

	}
	if (segCompleted.size() == 0)segCompleted.emplace_back(0U);

	if (print)
	{
		float sAverage = (float)sPts / (float)sCube;
		float sStandard = sqrt((float)sVar / (float)sCube - sAverage * sAverage);
		std::cout << "------------------------------------------------------------------------------\nStatistics\n";
		std::cout << "\nSum: " << sPts << "(" << std::fixed << setprecision(2) << (float)sPts / cloud_size * 100.0f
			<< "%), " << sPts / (r * (2 + EIGEN_PI) * seg * 10000.0f) << "pts/cm^2\n"
			<< "Cubes: " << sCube << "(Zero: " << sCube - sN << "(" << (float)(sCube - sN) / (float)sCube * 100.0f << "%), N: "
			<< sN << "(" << (float)sN / (float)sCube * 100.0f << "%))";
		std::cout << "\nμ = " << std::fixed << setprecision(3) << sAverage << ", σ = " << sStandard;
		std::cout << "\nnCompleted = " << std::fixed << setprecision(1) << nCompleted * precision << endl;
		for (auto i = segCompleted.begin(); i < segCompleted.end();)
		{
			auto it = check_continue(i);
			std::cout << setw(4) << *i * precision << " - " << setw(4) << *it * precision << "\n";
			i = it + 1;
		}
		std::cout << "\n------------------------------------------------------------------------------\n\n";

		std::string path = "";
		std::string filename = path + "l=" + std::to_string((unsigned)r) + "_" /*+ location*/ + ".csv";
		fstream opt(filename, std::ofstream::out | std::ofstream::app);
		if (!opt)
		{
			cout << "file failed" << endl;
		}
		opt << "\n, Sum, " << sPts << ", " << (float)sPts / cloud_size * 100.0f << "%, "
			<< sPts / (r * (2.0f + EIGEN_PI) * seg * 10000.0f) << "pts/cm^2, \n"
			<< ", Cubes, " << sCube << ", Zero, " << sCube - sN << ", " << (float)(sCube - sN) / sCube * 100.0f << "%, N, "
			<< sN << ", " << (float)sN / sCube * 100.0f << "%, \n"
			<< ", μ, " << sAverage << ", σ, " << sStandard << ", \n"
			<< ", nCompleted, " << nCompleted * precision << ", \n";
		opt.close();
	}

	delete cube;
	return (float)nCompleted * precision;
}


Arch::Arch() {}

float Arch::a = Arch::l - Arch::r;
float Arch::b = Arch::a * tan(Arch::alpha);
float Arch::R = sqrt(Arch::a * Arch::a + Arch::b * Arch::b) + Arch::r;
float Arch::ix = Arch::a + Arch::r * cos(Arch::alpha);
float Arch::iy = Arch::r * sin(Arch::alpha);
float Arch::f = Arch::R - Arch::b;

void Arch::arch(Eigen::Vector3d*& v, unsigned length) {
	cloud.width = length - 1 + cloud.size();
	cloud.height = 1;
	cloud.reserve(cloud.width * cloud.height);

	float ox = v[0].x(), oy = v[0].y(), oz = v[0].z();

	for (unsigned i = 1; i <= length; i++)
	{
		float tmpX = 0.0f, tmpY = 0.0f, tmpZ = 0.0f;
		float bx = v[i].x(), by = v[i].y(), bz = v[i].z();
		//先算跟大圓交點
		auto aR = 0.0f;
		auto bR = -b;
		//Ax+By+C, (x-a)^2+(y-b)^2=r^2
		float AR = (bx - ox) * (bx - ox) + (by - oy) * (by - oy);
		float BR = 2 * ((ox - aR) * (bx - ox) + (oy - bR) * (by - oy));
		float CR = (ox - aR) * (ox - aR) + (oy - bR) * (oy - bR) - R * R;
		float DR = BR * BR - 4 * AR * CR;

		if (DR > 0.0f)
		{
			float t1R = (-BR + sqrt(DR)) / (AR * 2)/*, t2 = (-BR - sqrt(DR)) / (AR * 2)*/;
			float tmpX1R = ox + (bx - ox) * t1R/*, tmpX2 = ox + (bx - ox) * t2*/;
			float tmpY1R = oy + (by - oy) * t1R/*, tmpY2 = oy + (by - oy) * t2*/;
			float tmpZ1R = oz + (bz - oz) * t1R/*, tmpZ2 = oz + (bz - oz) * t2*/;
			if (tmpY1R > 0.0f /*&& tmpZ1 >= 0*/)
			{
				tmpX = tmpX1R;
				tmpY = tmpY1R;
				tmpZ = tmpZ1R;
			}
			else if (tmpY1R < eps)
			{
				float t = oy / (oy - by);
				tmpX = ox + (bx - ox) * t;
				tmpY = 0.0f;
				tmpZ = oz + (bz - oz) * t;
			}

			//超過交點，大圓改小圓
			if (tmpY < iy /*&& abs(tmpX) > ix*/)
			{
				auto ar = (tmpX > 0) ? a : -a;
				auto br = 0;
				float Ar = (bx - ox) * (bx - ox) + (by - oy) * (by - oy);
				float Br = 2 * ((ox - ar) * (bx - ox) + (oy - br) * (by - oy));
				float Cr = (ox - ar) * (ox - ar) + (oy - br) * (oy - br) - r * r;
				float Dr = Br * Br - 4 * Ar * Cr;
				if (Dr > 0)
				{
					float t1r = (-Br + sqrt(Dr)) / (Ar * 2)/*, t2 = (-Br - sqrt(Dr)) / (Ar * 2)*/;
					float tmpX1r = ox + (bx - ox) * t1r/*, tmpX2 = ox + (bx - ox) * t2*/;
					float tmpY1r = oy + (by - oy) * t1r/*, tmpY2 = oy + (by - oy) * t2*/;
					float tmpZ1r = oz + (bz - oz) * t1r/*, tmpZ2 = oz + (bz - oz) * t2*/;
					if (tmpY1r > 0)
					{
						tmpX = tmpX1r;
						tmpY = tmpY1r;
						tmpZ = tmpZ1r;
					}
					else if (tmpY1r < eps)
					{
						float t = oy / (oy - by);
						tmpX = ox + (bx - ox) * t;
						tmpY = 0.0f;
						tmpZ = oz + (bz - oz) * t;
					}
				}
			}
			pcl::PointXYZL tmp{ tmpX,tmpY,tmpZ };
			cloud.emplace_back(tmp);
		}

	}
}

float Arch::statistics(bool print)
{
	cloud_cube.clear();
	cloud_cube.reserve((cube_x + cube_y) * 2 * cube_z);
	Cross_Section::cube_x = (unsigned)(l * 2.0f * scale) + 1U;
	Cross_Section::cube_y = (unsigned)(f * scale) + 1U;
	Cross_Section::cube_z = (unsigned)(seg * scale) + 1U;
	Cross_Section::nCube = cube_x * cube_y * cube_z;
	Cross_Section::cube = new unsigned[nCube];

	unsigned cloud_size = cloud.size();
	if (print)
	{
		std::cout << "------------------------------------------------------------------------------\n"
			<< "Number of points = " << cloud_size << endl;
		std::cout << "Number of cubes = " << cube_x << " * " << cube_y << " * " << cube_z << " = " << nCube
			<< "\nSegmentation: " << std::fixed << setprecision(2) << seg - start << "m, precision: " << precision << "m\n";
	}

#pragma omp parallel for num_threads(THREADS) schedule(dynamic, nCube/THREADS)
	for (unsigned i = 0; i < nCube; i++)
	{
		cube[i] = 0;
	}

#pragma omp parallel for num_threads(THREADS) schedule(dynamic, cloud_size/THREADS)
	for (size_t i = 0; i < cloud_size; i++)
	{
		//if (cloud.points.at(i).z > end || cloud.points.at(i).z < start)continue;
		if (cloud.points.at(i).z <= end && cloud.points.at(i).z >= start)
		{
			unsigned _x = (unsigned)((cloud.points.at(i).x + l) * scale);
			unsigned _y = (unsigned)(cloud.points.at(i).y * scale);
			unsigned _z = (unsigned)((cloud.points.at(i).z - start) * scale);
			++cube[_z * cube_x * cube_y + _x * cube_y + _y];
		}
	}


	auto sCube = 0ULL, sPts = 0ULL, sN = 0ULL, sVar = 0ULL;

	float threshold = precision * sqrt(2.0f);
	unsigned startScale = start * scale, endScale = end * scale;
	unsigned nCompleted = 0U, countCompleted = 0U;
	std::vector<unsigned> segCompleted;
	segCompleted.reserve(cube_z);

	for (unsigned k = 0; k < cube_z; k++)
	{
		float z_ = (float)(k + start * scale) * precision;
		unsigned completed = 0U;
		unsigned cubeCompleted = 0U;
		for (unsigned i = 0; i < cube_x; i++)
		{
			float x_ = (float)(i - l * scale) * precision;
			for (unsigned j = 0; j < cube_y; j++)
			{
				float y_ = (float)j * precision;
				if (abs(y_) <= eps)
				{
					if (print) {
						pcl::PointXYZL tmp{ x_,y_,z_ };
						cloud_cube.emplace_back(tmp);
					}
					++sCube;
					++cubeCompleted;
					auto n = (unsigned long long)cube[k * cube_x * cube_y + i * cube_y + j];
					if (n)
					{
						sVar += n * n;
						sPts += n;
						++sN;
						++completed;
					}
					continue;
				}

				float r_cube = 0.0;
				float r_after = 0.0;
				bool isEmpty = true;
				if (abs(x_) <= ix)
				{
					r_after = (x_ < 0)
						? sqrt((x_ + precision) * (x_ + precision) + (y_ + b) * (y_ + b))
						: sqrt(x_ * x_ + (y_ + b) * (y_ + b));
					if ((int&)R >= (int&)r_after && abs(R - r_after) - threshold <= eps)isEmpty = false;
				}
				else if (abs(x_) <= l)
				{
					auto aS = (x_ > ix) ? a : -a;
					r_after = (x_ < 0)
						? sqrt((x_ - aS + precision) * (x_ - aS + precision) + y_ * y_)
						: sqrt((x_ - aS) * (x_ - aS) + y_ * y_);
					if ((int&)r >= (int&)r_after && abs(r - r_after) - threshold <= eps)isEmpty = false;
				}

				if (!isEmpty)
				{
					if (print) {
						pcl::PointXYZL tmp{ x_,y_,z_ };
						cloud_cube.emplace_back(tmp);
					}
					++sCube;
					++cubeCompleted;
					auto n = (unsigned long long)cube[k * cube_x * cube_y + i * cube_y + j];
					if (n)
					{
						sVar += n * n;
						sPts += n;
						++sN;
						++completed;
					}
				}
			}
		}
		if ((float)completed / (float)cubeCompleted >= 0.9f)
		{
			++countCompleted;
			segCompleted.emplace_back((unsigned)(z_ * scale));
		}
		else
		{
			countCompleted = 0;
		}
		if (countCompleted > nCompleted)nCompleted = countCompleted;
	}

	if (segCompleted.size() == 0)segCompleted.emplace_back(0);

	if (print)
	{
		float sAverage = (float)sPts / sCube;
		float sStandard = sqrt((float)sVar / sCube - sAverage * sAverage);
		std::cout << "------------------------------------------------------------------------------\nStatistics\n";
		std::cout << "\nSum: " << sPts << "(" << std::fixed << setprecision(2) << (float)sPts / cloud_size * 100.0f
			<< "%), " << sPts / (r * (2 + EIGEN_PI) * seg * 10000.0) << "pts/cm^2\n"
			<< "Cubes: " << sCube << "(Zero: " << sCube - sN << "(" << (float)(sCube - sN) / sCube * 100.0f << "%), N: "
			<< sN << "(" << (float)sN / sCube * 100.0f << "%))";
		std::cout << "\nμ = " << std::fixed << setprecision(3) << sAverage << ", σ = " << sStandard;
		std::cout << "\nnCompleted = " << std::fixed << setprecision(1) << nCompleted * precision << endl;
		for (auto i = segCompleted.begin(); i < segCompleted.end();)
		{
			auto it = check_continue(i);
			std::cout << setw(4) << *i * precision << " - " << setw(4) << *it * precision << "\n";
			i = it + 1;
		}
		std::cout << "------------------------------------------------------------------------------\n\n";

		std::string path = "";
		std::string filename = path + "l=" + std::to_string((unsigned)l * 2) + "_" /*+ location*/ + ".csv";
		fstream opt(filename, std::ofstream::out | std::ofstream::app);
		if (!opt)
		{
			cout << "file failed" << endl;
		}
		opt << "\n, Sum, " << sPts << ", " << (float)sPts / cloud_size * 100.0f << "%, "
			<< sPts / (r * (2 + EIGEN_PI) * seg * 10000.0) << "pts/cm^2, \n"
			<< ", Cubes, " << sCube << ", Zero, " << sCube - sN << ", " << (float)(sCube - sN) / sCube * 100.0f << "%, N, "
			<< sN << ", " << (float)sN / sCube * 100.0f << "%, \n"
			<< ", μ, " << sAverage << ", σ, " << sStandard << ", \n"
			<< ", nCompleted, " << nCompleted * precision << ", \n";
		opt.close();


	}
	delete cube;
	return (float)nCompleted * precision;
}
