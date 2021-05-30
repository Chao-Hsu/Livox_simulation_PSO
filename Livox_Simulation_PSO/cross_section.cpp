#include "cross_section.h"

pcl::PointCloud<pcl::PointXYZ> cloud;

Cross_Section::Cross_Section() {}
float Cross_Section::d_to_r(float degree) { return degree / 180.0f * EIGEN_PI; }

Cylinder::Cylinder() {}

void Cylinder::cylinder(Eigen::Vector3d*& v, unsigned length) {
	cloud.width = length + cloud.size();
	cloud.height = 1;
	cloud.reserve(cloud.width * cloud.height);

	float ox = v[0].x(), oy = v[0].y(), oz = v[0].z();
	for (int i = 1; i <= length; i++)
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
			float t1 = (-B + sqrt(D)) / (A * 2.0f), t2 = (-B - sqrt(D)) / (A * 2.0f);
			float tmpX1 = ox + (bx - ox) * t1, tmpX2 = ox + (bx - ox) * t2;
			float tmpY1 = oy + (by - oy) * t1, tmpY2 = oy + (by - oy) * t2;
			float tmpZ1 = oz + (bz - oz) * t1, tmpZ2 = oz + (bz - oz) * t2;
			if (tmpY1 > 0)
			{
				tmpX = tmpX1;
				tmpY = tmpY1;
				tmpZ = tmpZ1;
			}
			else if (tmpY1 < eps || tmpY2 < eps)
			{
				float t = oy / (oy - by);
				tmpX = ox + (bx - ox) * t;
				tmpY = 0.0f;
				tmpZ = oz + (bz - oz) * t;
			}
			pcl::PointXYZ tmp{ tmpX,tmpY,tmpZ };
			cloud.emplace_back(tmp);
		}
	}
}

Arch::Arch() {}
void Arch::arch(Eigen::Vector3d*& v, unsigned length) {
	cloud.width = length + cloud.size();
	cloud.height = 1;
	cloud.reserve(cloud.width * cloud.height);

	a = l - r;
	b = a * tan(alpha);
	R = sqrt(a * a + b * b) + r;
	ix = a + r * cos(alpha);
	iy = r * sin(alpha);
	f = R - b;
	float ox = v[0].x(), oy = v[0].y(), oz = v[0].z();

	for (int i = 1; i <= length; i++)
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
			pcl::PointXYZ tmp{ tmpX,tmpY,tmpZ };
			cloud.emplace_back(tmp);
		}

	}
}