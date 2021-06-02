#pragma once

#include "Livox.h"

extern std::mt19937_64 generator;

class PSO
{
public:
	PSO();
	~PSO()
	{
		delete position;
		delete velocity;
		delete p_best_position;
		delete g_best_position;
		delete tmp_fitness;
		delete p_best_fitness;
	}
	void go();
	void move();
	void compute_f();
	float random(float lower, float upper);
private:
	float p_max[6];
	float p_min[6];
	float v_max[6];

	unsigned particle;
	float iteration;
	float limit;
	float W;
	float C1;
	float C2;

	unsigned para_length;
	unsigned pos_length;
	float* position;
	float* velocity;
	float* p_best_position;
	float* g_best_position;
	float* tmp_fitness;
	float* p_best_fitness;
	float g_best_fitness;
};

