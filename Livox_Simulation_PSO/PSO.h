#pragma once

#include "Livox.h"

extern std::random_device rd;
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
	void init();
	void set_p_max(float* p);
	void set_p_min(float* p);
private:
	void move();
	void compute_f();
	void update();
	float random(float lower, float upper);

	float p_max[6];
	float p_min[6];
	float v_max[6];

	unsigned particle;
	unsigned iteration;
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

