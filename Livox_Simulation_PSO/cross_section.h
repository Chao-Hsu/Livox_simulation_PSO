#pragma once

#include <iostream>
#include <iomanip>
#include <fstream>

#include <algorithm>
#include <string>
#include <random>

#include <ctime>
#include <cmath>
#include <cstdlib>

#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <omp.h>

extern int THREADS;
extern char flag_cross_section;
extern pcl::PointCloud<pcl::PointXYZ> cloud;


enum cross_section
{
	choose_cylinder = 1,
	choose_arch = 2
};

class Cross_Section
{
public:
	Cross_Section();
	~Cross_Section() {}
	static float d_to_r(float degree);

protected:
	constexpr static float eps = 1e-5;

};

class Cylinder : virtual public Cross_Section
{
public:
	Cylinder();
	~Cylinder() {}
	void cylinder(Eigen::Vector3d*& v, unsigned length);
private:
	constexpr static float r = 5.0f;
};

class Arch : virtual public Cross_Section
{
public:
	Arch();
	~Arch() {}
	void arch(Eigen::Vector3d*& v, unsigned length);
private:
	constexpr static float alpha = 45.0f / 180.0f * EIGEN_PI;
	constexpr static float l = 6.0f;
	constexpr static float r = 3.0f / 5.0f * l;
	float a;
	float b;
	float R;
	float ix;
	float iy;
	float f;

};