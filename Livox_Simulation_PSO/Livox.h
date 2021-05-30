#pragma once

#include "cross_section.h"

extern int nMid70;
extern int nHorizon;

class Livox :public Cylinder, public Arch
{
public:
	Livox();
	Livox(float tx, float ty, float tz, float rx, float ry);
	Livox(float tx, float ty, float tz, float rx, float ry, float rz);
	~Livox()
	{
		delete ray;
		std::cout << "delete Livox\n";
		//delete position;
		//delete velocity;
		//delete p_best_position;
		//delete g_best_position;
		//delete fitness;
		//delete p_best_fitness;
	}
	void print()const;

	//float get_scalar();
	//float get_ox();
	//float get_oy();
	//float get_oz();
	//float get_roll();
	//float get_pitch();

	//unsigned get_nPts();

	//float get_dt();
	//float get_w1();
	//float get_w2();

	//Eigen::Vector3d* get_ray();
protected:
	Eigen::Vector3d* ray;
	pcl::PointCloud<pcl::PointXYZ> cloud;


	float scalar;
	float ox;
	float oy;
	float oz;
	float roll;
	float pitch;

	unsigned scanning_time;
	unsigned nPtsPerSec;
	unsigned nPts;

	float dt;
	float rpm1;
	float rpm2;
	float w1;
	float w2;

	//int particle;
	//float iteration;
	//float limit;
	//float W;
	//float C1;
	//float C2;

	//int para_length;
	//float* position;
	//float* velocity;
	//float* p_best_position;
	//float* g_best_position;
	//float* fitness;
	//float* p_best_fitness;
	//float g_best_fitness;
};

class Mid : public Livox
{
public:
	Mid();
	Mid(float tx, float ty, float tz, float rx, float ry);
	~Mid() { std::cout << "delete Mid\n"; }
protected:
	float FoV;
};

class Mid70 : public Mid
{
public:
	Mid70();
	Mid70(float tx, float ty, float tz, float rx, float ry);
	~Mid70() { std::cout << "delete Mid70\n"; }
};

class Horizon : public Livox
{
public:
	Horizon();
	Horizon(float tx, float ty, float tz, float rx, float ry, float rz);
	~Horizon() { std::cout << "delete Horizon\n"; }
	//void set_FoV(float h, float v);
	//float get_FoV_h();
	//float get_FoV_v();

protected:
	float FoV_h;
	float FoV_v;

	float yaw;

	float rpm3;
	float w3;
	float n1;
	float n2;
	float alpha1;
	float alpha2;
	float delta1;
	float delta2;
	float rotation_ratio;
	float translation_ratio;
};